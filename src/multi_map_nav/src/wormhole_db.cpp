#include "multi_map_nav/wormhole_db.hpp"
#include <iostream>
#include <sstream>
#include <queue>
#include <map>

namespace multi_map_nav
{

WormholeDB::WormholeDB()
: db_(nullptr), logger_(rclcpp::get_logger("wormhole_db"))
{
}

WormholeDB::~WormholeDB()
{
  close();
}

bool WormholeDB::initialize(const std::string &db_path)
{
  RCLCPP_INFO(logger_, "Initializing wormhole database: %s", db_path.c_str());
  
  // Open database
  int rc = sqlite3_open(db_path.c_str(), &db_);
  if (rc != SQLITE_OK) {
    RCLCPP_ERROR(logger_, "Failed to open database: %s", sqlite3_errmsg(db_));
    sqlite3_close(db_);
    db_ = nullptr;
    return false;
  }
  
  // Create tables if they don't exist
  if (!create_tables()) {
    RCLCPP_ERROR(logger_, "Failed to create database tables");
    close();
    return false;
  }
  
  RCLCPP_INFO(logger_, "Wormhole database initialized successfully");
  return true;
}

bool WormholeDB::create_tables()
{
  const char* wormholes_table = 
    "CREATE TABLE IF NOT EXISTS wormholes ("
    "id INTEGER PRIMARY KEY AUTOINCREMENT,"
    "source_map TEXT NOT NULL,"
    "target_map TEXT NOT NULL,"
    "source_x REAL NOT NULL,"
    "source_y REAL NOT NULL,"
    "source_z REAL NOT NULL,"
    "source_qx REAL NOT NULL,"
    "source_qy REAL NOT NULL,"
    "source_qz REAL NOT NULL,"
    "source_qw REAL NOT NULL,"
    "target_x REAL NOT NULL,"
    "target_y REAL NOT NULL,"
    "target_z REAL NOT NULL,"
    "target_qx REAL NOT NULL,"
    "target_qy REAL NOT NULL,"
    "target_qz REAL NOT NULL,"
    "target_qw REAL NOT NULL,"
    "transition_cost REAL NOT NULL,"
    "UNIQUE(source_map, target_map));";

  const char* vda5050_nodes_table =
    "CREATE TABLE IF NOT EXISTS vda5050_nodes ("
    "node_id TEXT NOT NULL,"
    "sequence_id TEXT NOT NULL,"
    "map_id TEXT NOT NULL,"
    "pose_x REAL NOT NULL,"
    "pose_y REAL NOT NULL,"
    "pose_z REAL NOT NULL,"
    "pose_qx REAL NOT NULL,"
    "pose_qy REAL NOT NULL,"
    "pose_qz REAL NOT NULL,"
    "pose_qw REAL NOT NULL,"
    "released INTEGER NOT NULL,"
    "PRIMARY KEY(node_id, map_id));";

  const char* vda5050_edges_table =
    "CREATE TABLE IF NOT EXISTS vda5050_edges ("
    "edge_id TEXT NOT NULL,"
    "sequence_id TEXT NOT NULL,"
    "start_node_id TEXT NOT NULL,"
    "end_node_id TEXT NOT NULL,"
    "map_id TEXT NOT NULL,"
    "released INTEGER NOT NULL,"
    "PRIMARY KEY(edge_id, map_id));";

  const char* vda5050_orders_table =
    "CREATE TABLE IF NOT EXISTS vda5050_orders ("
    "order_id TEXT PRIMARY KEY,"
    "order_update_id INTEGER NOT NULL,"
    "order_data TEXT NOT NULL);";  // Store serialized order data as JSON

  const char* vda5050_actions_table =
    "CREATE TABLE IF NOT EXISTS vda5050_actions ("
    "node_id TEXT NOT NULL,"
    "map_id TEXT NOT NULL,"
    "action TEXT NOT NULL,"
    "PRIMARY KEY(node_id, map_id, action));";

  char* err_msg = nullptr;
  
  // Execute each table creation statement
  if (sqlite3_exec(db_, wormholes_table, nullptr, nullptr, &err_msg) != SQLITE_OK) {
    RCLCPP_ERROR(logger_, "SQL error creating wormholes table: %s", err_msg);
    sqlite3_free(err_msg);
    return false;
  }
  
  if (sqlite3_exec(db_, vda5050_nodes_table, nullptr, nullptr, &err_msg) != SQLITE_OK) {
    RCLCPP_ERROR(logger_, "SQL error creating VDA5050 nodes table: %s", err_msg);
    sqlite3_free(err_msg);
    return false;
  }
  
  if (sqlite3_exec(db_, vda5050_edges_table, nullptr, nullptr, &err_msg) != SQLITE_OK) {
    RCLCPP_ERROR(logger_, "SQL error creating VDA5050 edges table: %s", err_msg);
    sqlite3_free(err_msg);
    return false;
  }
  
  if (sqlite3_exec(db_, vda5050_orders_table, nullptr, nullptr, &err_msg) != SQLITE_OK) {
    RCLCPP_ERROR(logger_, "SQL error creating VDA5050 orders table: %s", err_msg);
    sqlite3_free(err_msg);
    return false;
  }
  
  if (sqlite3_exec(db_, vda5050_actions_table, nullptr, nullptr, &err_msg) != SQLITE_OK) {
    RCLCPP_ERROR(logger_, "SQL error creating VDA5050 actions table: %s", err_msg);
    sqlite3_free(err_msg);
    return false;
  }
  
  return true;
}

void WormholeDB::close()
{
  if (db_) {
    sqlite3_close(db_);
    db_ = nullptr;
    RCLCPP_INFO(logger_, "Wormhole database closed");
  }
}

bool WormholeDB::add_wormhole(const Wormhole& wormhole)
{
  if (!db_) {
    RCLCPP_ERROR(logger_, "Database not initialized");
    return false;
  }
  
  const char* sql = 
    "INSERT OR REPLACE INTO wormholes "
    "(source_map, target_map, source_x, source_y, source_z, source_qx, source_qy, source_qz, source_qw, "
    "target_x, target_y, target_z, target_qx, target_qy, target_qz, target_qw, transition_cost) "
    "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);";
  
  sqlite3_stmt* stmt;
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    RCLCPP_ERROR(logger_, "Failed to prepare statement: %s", sqlite3_errmsg(db_));
    return false;
  }
  
  sqlite3_bind_text(stmt, 1, wormhole.source_map.c_str(), -1, SQLITE_STATIC);
  sqlite3_bind_text(stmt, 2, wormhole.target_map.c_str(), -1, SQLITE_STATIC);
  sqlite3_bind_double(stmt, 3, wormhole.source_pose.position.x);
  sqlite3_bind_double(stmt, 4, wormhole.source_pose.position.y);
  sqlite3_bind_double(stmt, 5, wormhole.source_pose.position.z);
  sqlite3_bind_double(stmt, 6, wormhole.source_pose.orientation.x);
  sqlite3_bind_double(stmt, 7, wormhole.source_pose.orientation.y);
  sqlite3_bind_double(stmt, 8, wormhole.source_pose.orientation.z);
  sqlite3_bind_double(stmt, 9, wormhole.source_pose.orientation.w);
  sqlite3_bind_double(stmt, 10, wormhole.target_pose.position.x);
  sqlite3_bind_double(stmt, 11, wormhole.target_pose.position.y);
  sqlite3_bind_double(stmt, 12, wormhole.target_pose.position.z);
  sqlite3_bind_double(stmt, 13, wormhole.target_pose.orientation.x);
  sqlite3_bind_double(stmt, 14, wormhole.target_pose.orientation.y);
  sqlite3_bind_double(stmt, 15, wormhole.target_pose.orientation.z);
  sqlite3_bind_double(stmt, 16, wormhole.target_pose.orientation.w);
  sqlite3_bind_double(stmt, 17, wormhole.transition_cost);
  
  bool success = sqlite3_step(stmt) == SQLITE_DONE;
  if (!success) {
    RCLCPP_ERROR(logger_, "Failed to add wormhole: %s", sqlite3_errmsg(db_));
  }
  
  sqlite3_finalize(stmt);
  return success;
}

bool WormholeDB::add_vda5050_node(const vda5050::Node& node)
{
  if (!db_) {
    RCLCPP_ERROR(logger_, "Database not initialized");
    return false;
  }
  
  // Start a transaction since we'll do multiple operations
  sqlite3_exec(db_, "BEGIN TRANSACTION", nullptr, nullptr, nullptr);
  
  // Insert the node
  const char* sql = 
    "INSERT OR REPLACE INTO vda5050_nodes "
    "(node_id, sequence_id, map_id, pose_x, pose_y, pose_z, pose_qx, pose_qy, pose_qz, pose_qw, released) "
    "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);";
  
  sqlite3_stmt* stmt;
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    RCLCPP_ERROR(logger_, "Failed to prepare statement: %s", sqlite3_errmsg(db_));
    sqlite3_exec(db_, "ROLLBACK", nullptr, nullptr, nullptr);
    return false;
  }
  
  sqlite3_bind_text(stmt, 1, node.node_id.c_str(), -1, SQLITE_STATIC);
  sqlite3_bind_text(stmt, 2, node.sequence_id.c_str(), -1, SQLITE_STATIC);
  sqlite3_bind_text(stmt, 3, node.map_id.c_str(), -1, SQLITE_STATIC);
  sqlite3_bind_double(stmt, 4, node.pose.position.x);
  sqlite3_bind_double(stmt, 5, node.pose.position.y);
  sqlite3_bind_double(stmt, 6, node.pose.position.z);
  sqlite3_bind_double(stmt, 7, node.pose.orientation.x);
  sqlite3_bind_double(stmt, 8, node.pose.orientation.y);
  sqlite3_bind_double(stmt, 9, node.pose.orientation.z);
  sqlite3_bind_double(stmt, 10, node.pose.orientation.w);
  sqlite3_bind_int(stmt, 11, node.released ? 1 : 0);
  
  bool success = sqlite3_step(stmt) == SQLITE_DONE;
  sqlite3_finalize(stmt);
  
  if (!success) {
    RCLCPP_ERROR(logger_, "Failed to add VDA5050 node: %s", sqlite3_errmsg(db_));
    sqlite3_exec(db_, "ROLLBACK", nullptr, nullptr, nullptr);
    return false;
  }
  
  // Insert node actions if any
  for (const auto& action : node.actions) {
    const char* action_sql = 
      "INSERT OR REPLACE INTO vda5050_actions (node_id, map_id, action) VALUES (?, ?, ?);";
    
    if (sqlite3_prepare_v2(db_, action_sql, -1, &stmt, nullptr) != SQLITE_OK) {
      RCLCPP_ERROR(logger_, "Failed to prepare statement: %s", sqlite3_errmsg(db_));
      sqlite3_exec(db_, "ROLLBACK", nullptr, nullptr, nullptr);
      return false;
    }
    
    sqlite3_bind_text(stmt, 1, node.node_id.c_str(), -1, SQLITE_STATIC);
    sqlite3_bind_text(stmt, 2, node.map_id.c_str(), -1, SQLITE_STATIC);
    sqlite3_bind_text(stmt, 3, action.c_str(), -1, SQLITE_STATIC);
    
    success = sqlite3_step(stmt) == SQLITE_DONE;
    sqlite3_finalize(stmt);
    
    if (!success) {
      RCLCPP_ERROR(logger_, "Failed to add VDA5050 action: %s", sqlite3_errmsg(db_));
      sqlite3_exec(db_, "ROLLBACK", nullptr, nullptr, nullptr);
      return false;
    }
  }
  
  // Commit the transaction
  sqlite3_exec(db_, "COMMIT", nullptr, nullptr, nullptr);
  return true;
}

bool WormholeDB::store_vda5050_order(const vda5050::Order& order)
{
  if (!db_) {
    RCLCPP_ERROR(logger_, "Database not initialized");
    return false;
  }
  
  // Create JSON string representation of the order
  std::string order_data = "{"; // In a real implementation, use nlohmann::json to serialize properly
  order_data += "\"orderId\":\"" + order.order_id + "\",";
  order_data += "\"orderUpdateId\":" + std::to_string(order.order_update_id) + ",";
  order_data += "\"nodes\":[],"; // Simplified for implementation
  order_data += "\"edges\":[]";   // Simplified for implementation
  order_data += "}";
  
  const char* sql = 
    "INSERT OR REPLACE INTO vda5050_orders "
    "(order_id, order_update_id, order_data) "
    "VALUES (?, ?, ?);";
  
  sqlite3_stmt* stmt;
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    RCLCPP_ERROR(logger_, "Failed to prepare statement: %s", sqlite3_errmsg(db_));
    return false;
  }
  
  sqlite3_bind_text(stmt, 1, order.order_id.c_str(), -1, SQLITE_STATIC);
  sqlite3_bind_int(stmt, 2, order.order_update_id);
  sqlite3_bind_text(stmt, 3, order_data.c_str(), -1, SQLITE_STATIC);
  
  bool success = sqlite3_step(stmt) == SQLITE_DONE;
  if (!success) {
    RCLCPP_ERROR(logger_, "Failed to store VDA5050 order: %s", sqlite3_errmsg(db_));
  }
  
  sqlite3_finalize(stmt);
  return success;
}

bool WormholeDB::get_vda5050_order(const std::string& order_id, vda5050::Order& order)
{
  if (!db_) {
    RCLCPP_ERROR(logger_, "Database not initialized");
    return false;
  }
  
  const char* sql = 
    "SELECT order_id, order_update_id, order_data "
    "FROM vda5050_orders "
    "WHERE order_id = ?;";
  
  sqlite3_stmt* stmt;
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    RCLCPP_ERROR(logger_, "Failed to prepare statement: %s", sqlite3_errmsg(db_));
    return false;
  }
  
  sqlite3_bind_text(stmt, 1, order_id.c_str(), -1, SQLITE_STATIC);
  
  bool success = false;
  if (sqlite3_step(stmt) == SQLITE_ROW) {
    order.order_id = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
    order.order_update_id = sqlite3_column_int(stmt, 1);
    // In a real implementation, parse the JSON from column 2 (order_data) 
    // to populate the nodes and edges vectors
    success = true;
  }
  
  sqlite3_finalize(stmt);
  return success;
}

bool WormholeDB::get_vda5050_nodes(const std::string& map_id, std::vector<vda5050::Node>& nodes)
{
  if (!db_) {
    RCLCPP_ERROR(logger_, "Database not initialized");
    return false;
  }
  
  const char* sql = 
    "SELECT node_id, sequence_id, pose_x, pose_y, pose_z, pose_qx, pose_qy, pose_qz, pose_qw, released "
    "FROM vda5050_nodes "
    "WHERE map_id = ?;";
  
  sqlite3_stmt* stmt;
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    RCLCPP_ERROR(logger_, "Failed to prepare statement: %s", sqlite3_errmsg(db_));
    return false;
  }
  
  sqlite3_bind_text(stmt, 1, map_id.c_str(), -1, SQLITE_STATIC);
  
  nodes.clear();
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    vda5050::Node node;
    node.node_id = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
    node.sequence_id = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
    node.map_id = map_id;
    
    node.pose.position.x = sqlite3_column_double(stmt, 2);
    node.pose.position.y = sqlite3_column_double(stmt, 3);
    node.pose.position.z = sqlite3_column_double(stmt, 4);
    node.pose.orientation.x = sqlite3_column_double(stmt, 5);
    node.pose.orientation.y = sqlite3_column_double(stmt, 6);
    node.pose.orientation.z = sqlite3_column_double(stmt, 7);
    node.pose.orientation.w = sqlite3_column_double(stmt, 8);
    
    node.released = sqlite3_column_int(stmt, 9) != 0;
    
    // Get actions for this node
    const char* action_sql = 
      "SELECT action FROM vda5050_actions WHERE node_id = ? AND map_id = ?;";
    
    sqlite3_stmt* action_stmt;
    if (sqlite3_prepare_v2(db_, action_sql, -1, &action_stmt, nullptr) == SQLITE_OK) {
      sqlite3_bind_text(action_stmt, 1, node.node_id.c_str(), -1, SQLITE_STATIC);
      sqlite3_bind_text(action_stmt, 2, map_id.c_str(), -1, SQLITE_STATIC);
      
      while (sqlite3_step(action_stmt) == SQLITE_ROW) {
        node.actions.push_back(reinterpret_cast<const char*>(sqlite3_column_text(action_stmt, 0)));
      }
      
      sqlite3_finalize(action_stmt);
    }
    
    nodes.push_back(node);
  }
  
  sqlite3_finalize(stmt);
  return true;
}

bool WormholeDB::remove_wormhole(const std::string& source_map, const std::string& target_map)
{
  if (!db_) {
    RCLCPP_ERROR(logger_, "Database not initialized");
    return false;
  }
  
  const char* sql = 
    "DELETE FROM wormholes WHERE source_map = ? AND target_map = ?;";
  
  sqlite3_stmt* stmt;
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    RCLCPP_ERROR(logger_, "Failed to prepare statement: %s", sqlite3_errmsg(db_));
    return false;
  }
  
  sqlite3_bind_text(stmt, 1, source_map.c_str(), -1, SQLITE_STATIC);
  sqlite3_bind_text(stmt, 2, target_map.c_str(), -1, SQLITE_STATIC);
  
  bool success = sqlite3_step(stmt) == SQLITE_DONE;
  if (!success) {
    RCLCPP_ERROR(logger_, "Failed to remove wormhole: %s", sqlite3_errmsg(db_));
  }
  
  sqlite3_finalize(stmt);
  return success;
}

bool WormholeDB::get_wormholes(const std::string& map_name, std::vector<Wormhole>& wormholes)
{
  if (!db_) {
    RCLCPP_ERROR(logger_, "Database not initialized");
    return false;
  }
  
  const char* sql = 
    "SELECT source_map, target_map, "
    "source_x, source_y, source_z, source_qx, source_qy, source_qz, source_qw, "
    "target_x, target_y, target_z, target_qx, target_qy, target_qz, target_qw, "
    "transition_cost "
    "FROM wormholes WHERE source_map = ?;";
  
  sqlite3_stmt* stmt;
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    RCLCPP_ERROR(logger_, "Failed to prepare statement: %s", sqlite3_errmsg(db_));
    return false;
  }
  
  sqlite3_bind_text(stmt, 1, map_name.c_str(), -1, SQLITE_STATIC);
  
  wormholes.clear();
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    Wormhole wormhole;
    wormhole.source_map = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
    wormhole.target_map = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
    
    wormhole.source_pose.position.x = sqlite3_column_double(stmt, 2);
    wormhole.source_pose.position.y = sqlite3_column_double(stmt, 3);
    wormhole.source_pose.position.z = sqlite3_column_double(stmt, 4);
    wormhole.source_pose.orientation.x = sqlite3_column_double(stmt, 5);
    wormhole.source_pose.orientation.y = sqlite3_column_double(stmt, 6);
    wormhole.source_pose.orientation.z = sqlite3_column_double(stmt, 7);
    wormhole.source_pose.orientation.w = sqlite3_column_double(stmt, 8);
    
    wormhole.target_pose.position.x = sqlite3_column_double(stmt, 9);
    wormhole.target_pose.position.y = sqlite3_column_double(stmt, 10);
    wormhole.target_pose.position.z = sqlite3_column_double(stmt, 11);
    wormhole.target_pose.orientation.x = sqlite3_column_double(stmt, 12);
    wormhole.target_pose.orientation.y = sqlite3_column_double(stmt, 13);
    wormhole.target_pose.orientation.z = sqlite3_column_double(stmt, 14);
    wormhole.target_pose.orientation.w = sqlite3_column_double(stmt, 15);
    
    wormhole.transition_cost = sqlite3_column_double(stmt, 16);
    
    wormholes.push_back(wormhole);
  }
  
  sqlite3_finalize(stmt);
  return true;
}

bool WormholeDB::find_path(const std::string& start_map, const geometry_msgs::msg::Pose& /*start_pose*/,
                          const std::string& goal_map, const geometry_msgs::msg::Pose& /*goal_pose*/,
                          std::vector<Wormhole>& path)
{
  if (!db_) {
    RCLCPP_ERROR(logger_, "Database not initialized");
    return false;
  }
  
  path.clear();
  
  // Same map - no wormholes needed
  if (start_map == goal_map) {
    return true;
  }
  
  // Try direct connection first
  const char* direct_sql = 
    "SELECT source_map, target_map, "
    "source_x, source_y, source_z, source_qx, source_qy, source_qz, source_qw, "
    "target_x, target_y, target_z, target_qx, target_qy, target_qz, target_qw, "
    "transition_cost "
    "FROM wormholes WHERE source_map = ? AND target_map = ?;";
  
  sqlite3_stmt* stmt;
  if (sqlite3_prepare_v2(db_, direct_sql, -1, &stmt, nullptr) != SQLITE_OK) {
    RCLCPP_ERROR(logger_, "Failed to prepare statement: %s", sqlite3_errmsg(db_));
    return false;
  }
  
  sqlite3_bind_text(stmt, 1, start_map.c_str(), -1, SQLITE_STATIC);
  sqlite3_bind_text(stmt, 2, goal_map.c_str(), -1, SQLITE_STATIC);
  
  // Check for direct path
  if (sqlite3_step(stmt) == SQLITE_ROW) {
    Wormhole wormhole;
    wormhole.source_map = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
    wormhole.target_map = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
    
    wormhole.source_pose.position.x = sqlite3_column_double(stmt, 2);
    wormhole.source_pose.position.y = sqlite3_column_double(stmt, 3);
    wormhole.source_pose.position.z = sqlite3_column_double(stmt, 4);
    wormhole.source_pose.orientation.x = sqlite3_column_double(stmt, 5);
    wormhole.source_pose.orientation.y = sqlite3_column_double(stmt, 6);
    wormhole.source_pose.orientation.z = sqlite3_column_double(stmt, 7);
    wormhole.source_pose.orientation.w = sqlite3_column_double(stmt, 8);
    
    wormhole.target_pose.position.x = sqlite3_column_double(stmt, 9);
    wormhole.target_pose.position.y = sqlite3_column_double(stmt, 10);
    wormhole.target_pose.position.z = sqlite3_column_double(stmt, 11);
    wormhole.target_pose.orientation.x = sqlite3_column_double(stmt, 12);
    wormhole.target_pose.orientation.y = sqlite3_column_double(stmt, 13);
    wormhole.target_pose.orientation.z = sqlite3_column_double(stmt, 14);
    wormhole.target_pose.orientation.w = sqlite3_column_double(stmt, 15);
    
    wormhole.transition_cost = sqlite3_column_double(stmt, 16);
    
    path.push_back(wormhole);
    sqlite3_finalize(stmt);
    return true;
  }
  
  sqlite3_finalize(stmt);
  
  // No direct path found, try to find a multi-hop path
  // Using breadth-first search (BFS) to find the shortest path
  
  // Maps we've visited and their predecessor in the path
  std::map<std::string, std::string> visited;
  visited[start_map] = "";  // Start map has no predecessor
  
  // Queue for BFS
  std::queue<std::string> queue;
  queue.push(start_map);
  
  // Helper map for edges (key: source+target, value: wormhole info)
  std::map<std::string, Wormhole> edges;
  
  // BFS loop
  bool found_path = false;
  while (!queue.empty() && !found_path) {
    std::string current_map = queue.front();
    queue.pop();
    
    // Get all wormholes from current_map
    std::vector<Wormhole> outgoing_wormholes;
    get_wormholes(current_map, outgoing_wormholes);
    
    for (const auto& wormhole : outgoing_wormholes) {
      const std::string& next_map = wormhole.target_map;
      
      // Store the edge information
      std::string edge_key = wormhole.source_map + "->" + wormhole.target_map;
      edges[edge_key] = wormhole;
      
      // Check if we reached the goal
      if (next_map == goal_map) {
        visited[next_map] = current_map;
        found_path = true;
        break;
      }
      
      // Check if we've already visited this map
      if (visited.find(next_map) == visited.end()) {
        visited[next_map] = current_map;
        queue.push(next_map);
      }
    }
  }
  
  // If we found a path, reconstruct it
  if (found_path) {
    std::vector<Wormhole> reverse_path;
    std::string current = goal_map;
    
    // Trace back from goal to start
    while (current != start_map) {
      std::string previous = visited[current];
      std::string edge_key = previous + "->" + current;
      reverse_path.push_back(edges[edge_key]);
      current = previous;
    }
    
    // Reverse the path to get start-to-goal order
    for (auto it = reverse_path.rbegin(); it != reverse_path.rend(); ++it) {
      path.push_back(*it);
    }
    
    RCLCPP_INFO(logger_, "Found multi-hop path with %zu wormholes", path.size());
    return true;
  }
  
  RCLCPP_ERROR(logger_, "No path found between %s and %s", start_map.c_str(), goal_map.c_str());
  return false;
}

}  // namespace multi_map_nav
