#ifndef MULTI_MAP_NAV_WORMHOLE_DB_HPP
#define MULTI_MAP_NAV_WORMHOLE_DB_HPP

#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <sqlite3.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "multi_map_nav/vda5050_types.hpp"

namespace multi_map_nav
{

/**
 * @struct Wormhole
 * @brief Represents a connection (wormhole) between two separate maps
 * 
 * A wormhole defines a transition point between two separate maps. When a robot
 * needs to navigate across different maps, it uses these wormholes to determine
 * the path and transition points. Each wormhole has a source map and position,
 * and a target map and position.
 */
struct Wormhole {
  std::string source_map;      ///< Name of the source map
  std::string target_map;      ///< Name of the target map
  geometry_msgs::msg::Pose source_pose;  ///< Position in the source map
  geometry_msgs::msg::Pose target_pose;  ///< Position in the target map
  float transition_cost;       ///< Cost associated with this transition
};

/**
 * @class WormholeDB
 * @brief SQLite database interface for storing and retrieving wormhole data
 * 
 * This class provides an interface to the SQLite database that stores wormhole
 * information for multi-map navigation. It handles database operations including:
 * - Creating and initializing database tables
 * - Adding and removing wormholes
 * - Querying wormholes for navigation
 * - Finding paths between maps
 * - Storing VDA5050 order information
 * 
 * The class follows SOLID principles with:
 * - Single Responsibility: Handles only database operations
 * - Open/Closed: Extensible through additional query methods
 * - Liskov Substitution: Can be replaced with any implementation of the same interface
 * - Interface Segregation: Methods grouped by functionality
 * - Dependency Inversion: Depends on abstractions (SQL interface) not details
 */
class WormholeDB
{
public:
  /**
   * @brief Constructor for WormholeDB
   */
  WormholeDB();
  
  /**
   * @brief Destructor for WormholeDB, ensures database is closed
   */
  ~WormholeDB();

  /**
   * @brief Initialize the database connection
   * @param db_path Path to the SQLite database file
   * @return true if initialization was successful, false otherwise
   */
  bool initialize(const std::string &db_path);
  
  /**
   * @brief Close the database connection
   */
  void close();

  /**
   * @brief Add a new wormhole to the database
   * @param wormhole The wormhole object to add
   * @return true if the operation was successful, false otherwise
   */
  bool add_wormhole(const Wormhole& wormhole);
  
  /**
   * @brief Remove a wormhole from the database
   * @param source_map Source map name
   * @param target_map Target map name
   * @return true if the operation was successful, false otherwise
   */
  bool remove_wormhole(const std::string& source_map, const std::string& target_map);
  
  /**
   * @brief Get all wormholes for a specific map
   * @param map_name Map name to query wormholes for
   * @param[out] wormholes Vector to store the retrieved wormholes
   * @return true if the operation was successful, false otherwise
   */
  bool get_wormholes(const std::string& map_name, std::vector<Wormhole>& wormholes);
  
  /**
   * @brief Find a path between two maps using wormholes
   * 
   * This method uses a breadth-first search algorithm to find the shortest path
   * between two maps using the available wormholes.
   * 
   * @param start_map Starting map name
   * @param start_pose Starting pose in the start map
   * @param goal_map Goal map name
   * @param goal_pose Goal pose in the goal map
   * @param[out] path Vector to store the sequence of wormholes to traverse
   * @return true if a path was found, false otherwise
   */
  bool find_path(const std::string& start_map, const geometry_msgs::msg::Pose& start_pose,
                 const std::string& goal_map, const geometry_msgs::msg::Pose& goal_pose,
                 std::vector<Wormhole>& path);

  /**
   * @brief Calculate the cost of a given wormhole path
   * @param path Path of wormholes
   * @return Total cost of the path
   */
  float calculate_path_cost(const std::vector<Wormhole>& path) const;

  /**
   * @brief Get maps connected to a specific map
   * @param map_name Map name to find connections for
   * @param[out] connected_maps Vector to store the connected map names
   * @return true if the operation was successful, false otherwise
   */
  bool get_connected_maps(const std::string& map_name, std::vector<std::string>& connected_maps);

  /**
   * @brief Store a robot trajectory for visualization
   * @param map_id Map where the trajectory was recorded
   * @param trajectory Vector of poses representing the trajectory
   * @param timestamp Timestamp for when the trajectory was recorded
   * @return true if the operation was successful, false otherwise
   */
  bool store_trajectory(const std::string& map_id, 
                        const std::vector<geometry_msgs::msg::Pose>& trajectory,
                        const std::chrono::system_clock::time_point& timestamp);

  /**
   * @brief Retrieve stored trajectories for visualization
   * @param map_id Map to get trajectories for (empty for all maps)
   * @param[out] trajectories Map of map_id to trajectory poses
   * @param start_time Start time for trajectory filtering (optional)
   * @param end_time End time for trajectory filtering (optional)
   * @return true if the operation was successful, false otherwise
   */
  bool get_trajectories(const std::string& map_id,
                        std::map<std::string, std::vector<geometry_msgs::msg::Pose>>& trajectories,
                        const std::chrono::system_clock::time_point* start_time = nullptr,
                        const std::chrono::system_clock::time_point* end_time = nullptr);

  /**
   * @brief Add a VDA5050 node to the database
   * @param node The VDA5050 node to add
   * @return true if the operation was successful, false otherwise
   */
  bool add_vda5050_node(const vda5050::Node& node);
  
  /**
   * @brief Get all VDA5050 nodes for a specific map
   * @param map_id Map ID to query nodes for
   * @param[out] nodes Vector to store the retrieved nodes
   * @return true if the operation was successful, false otherwise
   */
  bool get_vda5050_nodes(const std::string& map_id, std::vector<vda5050::Node>& nodes);
  
  /**
   * @brief Store a VDA5050 order in the database
   * @param order The VDA5050 order to store
   * @return true if the operation was successful, false otherwise
   */
  bool store_vda5050_order(const vda5050::Order& order);
  
  /**
   * @brief Get a VDA5050 order from the database
   * @param order_id Order ID to query
   * @param[out] order Order object to populate
   * @return true if the operation was successful, false otherwise
   */
  bool get_vda5050_order(const std::string& order_id, vda5050::Order& order);

private:
  /**
   * @brief Create database tables if they don't exist
   * @return true if successful, false otherwise
   */
  bool create_tables();

  /**
   * @brief Create database table for storing robot trajectories
   * @return true if successful, false otherwise
   */
  bool create_trajectory_table();
  
  sqlite3 *db_;          ///< SQLite database handle
  rclcpp::Logger logger_; ///< ROS logger
};

}  // namespace multi_map_nav

#endif  // MULTI_MAP_NAV_WORMHOLE_DB_HPP
