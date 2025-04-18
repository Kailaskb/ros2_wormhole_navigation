/**
 * @file vda5050_types.hpp
 * @brief VDA 5050 type definitions for multi-map navigation
 * 
 * This file defines data structures for working with the VDA 5050 protocol,
 * which is a standard for AGV communication in industrial environments.
 * The types defined here are used to interface with fleet management systems
 * that support the VDA 5050 protocol.
 */

#ifndef MULTI_MAP_NAV_VDA5050_TYPES_HPP
#define MULTI_MAP_NAV_VDA5050_TYPES_HPP

#include <string>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>

namespace multi_map_nav
{
namespace vda5050
{

/**
 * @enum ActionType
 * @brief Enumeration of supported VDA 5050 action types
 */
enum class ActionType {
  PICK,       ///< Pick an object
  DROP,       ///< Drop an object
  WAITING,    ///< Wait at a location
  CHARGING,   ///< Charge the robot
  CUSTOM      ///< Custom action
};

/**
 * @struct Node
 * @brief VDA 5050 node representation
 * 
 * In VDA 5050, nodes represent waypoints for the robot to visit.
 * Each node has a unique ID, a map ID, a position, and potentially
 * actions to perform at that node.
 */
struct Node {
  std::string node_id;    ///< Unique node identifier
  std::string sequence_id; ///< Sequence ID for ordering
  std::string map_id;     ///< Map identifier
  geometry_msgs::msg::Pose pose; ///< Position and orientation
  std::vector<std::string> actions; ///< Actions to perform at this node
  bool released;          ///< Whether this node is released for execution
};

/**
 * @struct Edge
 * @brief VDA 5050 edge representation
 * 
 * In VDA 5050, edges represent connections between nodes.
 * They define how the robot should travel between waypoints.
 */
struct Edge {
  std::string edge_id;    ///< Unique edge identifier
  std::string sequence_id; ///< Sequence ID for ordering
  std::string start_node_id; ///< ID of the starting node
  std::string end_node_id; ///< ID of the ending node
  std::string map_id;     ///< Map identifier
  bool released;          ///< Whether this edge is released for execution
};

/**
 * @struct Order
 * @brief VDA 5050 order representation
 * 
 * An order is a complete navigation task composed of nodes and edges.
 * It defines a full path for the robot to follow, including any actions
 * to perform along the way.
 */
struct Order {
  std::string order_id;    ///< Unique order identifier
  int order_update_id;     ///< Version number for this order
  std::vector<Node> nodes; ///< Nodes (waypoints) in this order
  std::vector<Edge> edges; ///< Edges (connections) in this order
};

}  // namespace vda5050
}  // namespace multi_map_nav

#endif  // MULTI_MAP_NAV_VDA5050_TYPES_HPP
