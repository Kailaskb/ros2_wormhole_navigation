/**
 * @file navigation_server.cpp
 * @brief Implementation of the multi-map navigation action server
 * 
 * This file implements the main navigation action server for multi-map navigation.
 * The server handles navigation goals across multiple maps, transitioning through
 * wormholes as necessary to reach the goal position.
 * 
 * The implementation follows a modular design pattern, separating concerns into:
 * - Goal handling (accepting, rejecting, cancelling)
 * - Path planning across maps
 * - Map transition management
 * - Navigation execution
 * - VDA5050 integration
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include "multi_map_nav/wormhole_db.hpp"
#include "multi_map_nav/action/multi_map_navigate.hpp"
#include "multi_map_nav/vda5050_types.hpp"

/**
 * @class MultiMapNavigationServer
 * @brief Action server for handling multi-map navigation requests
 * 
 * This class implements a ROS2 action server that accepts navigation goals
 * specifying a target map and position. It handles navigation between different
 * maps by:
 * 1. Finding a path through wormholes if the target is on a different map
 * 2. Navigating to wormholes and transitioning between maps as needed
 * 3. Publishing the current map name and providing navigation feedback
 * 
 * It also provides an interface to the VDA5050 protocol for integration with
 * industrial fleet management systems.
 */
class MultiMapNavigationServer : public rclcpp::Node
{
public:
  using MultiMapNavigate = multi_map_nav::action::MultiMapNavigate;
  using GoalHandleMultiMapNavigate = rclcpp_action::ServerGoalHandle<MultiMapNavigate>;
  
  /**
   * @brief Constructor for MultiMapNavigationServer
   * 
   * Initializes the database connection, publishers, subscribers, and action server.
   */
  MultiMapNavigationServer()
  : Node("multi_map_navigation_server")
  {
    // Declare parameters
    this->declare_parameter("db_path", "/tmp/wormhole.db");
    this->declare_parameter("current_map", "room1");
    
    // Get parameters
    std::string db_path = this->get_parameter("db_path").as_string();
    current_map_ = this->get_parameter("current_map").as_string();
    
    // Initialize wormhole database
    db_.initialize(db_path);
    
    // Publishers for VDA 5050 interface
    vda_state_pub_ = this->create_publisher<std_msgs::msg::String>("/vda5050/state", 10);
    vda_visualization_pub_ = this->create_publisher<nav_msgs::msg::Path>("/vda5050/visualization", 10);
    
    // Publishers for map management
    current_map_pub_ = this->create_publisher<std_msgs::msg::String>("/multi_map_nav/current_map", 10);
    
    // Subscribers for VDA 5050 interface
    vda_order_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/vda5050/order", 10,
      std::bind(&MultiMapNavigationServer::handle_vda_order, this, std::placeholders::_1));
    
    // Register action server
    action_server_ = rclcpp_action::create_server<MultiMapNavigate>(
      this,
      "multi_map_navigate",
      std::bind(&MultiMapNavigationServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MultiMapNavigationServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MultiMapNavigationServer::handle_accepted, this, std::placeholders::_1)
    );
    
    // Service clients for map management
    map_change_client_ = this->create_client<std_srvs::srv::Empty>("/map_server/map_change");
    
    // Timer for VDA 5050 state updates
    vda_state_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MultiMapNavigationServer::publish_vda_state, this));
    
    // Initial map publication
    auto map_msg = std::make_unique<std_msgs::msg::String>();
    map_msg->data = current_map_;
    current_map_pub_->publish(std::move(map_msg));
    
    RCLCPP_INFO(this->get_logger(), "Multi-map navigation server started with map: %s", current_map_.c_str());
  }

private:
  multi_map_nav::WormholeDB db_;
  rclcpp_action::Server<MultiMapNavigate>::SharedPtr action_server_;
  std::string current_map_;
  
  // VDA 5050 support
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vda_state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr vda_visualization_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vda_order_sub_;
  rclcpp::TimerBase::SharedPtr vda_state_timer_;
  multi_map_nav::vda5050::Order current_order_;
  
  // Map management
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr current_map_pub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr map_change_client_;
  
  /**
   * @brief Change the current map
   * @param new_map Name of the new map to switch to
   * 
   * This function handles the map switching process:
   * 1. Updates the current_map_ variable
   * 2. Publishes the map change to other nodes
   * 3. Triggers any necessary map server updates
   */
  void change_map(const std::string& new_map) {
    if (current_map_ == new_map) {
      return;  // No change needed
    }
    
    RCLCPP_INFO(this->get_logger(), "Changing map from %s to %s", current_map_.c_str(), new_map.c_str());
    current_map_ = new_map;
    
    // Publish the map change
    auto map_msg = std::make_unique<std_msgs::msg::String>();
    map_msg->data = current_map_;
    current_map_pub_->publish(std::move(map_msg));
    
    // Trigger map server update in a real implementation
    // This would involve more sophisticated map switching depending on the navigation stack
    if (map_change_client_->service_is_ready()) {
      auto request = std::make_shared<std_srvs::srv::Empty::Request>();
      RCLCPP_INFO(this->get_logger(), "Requesting map change service");
      map_change_client_->async_send_request(request);
    } else {
      RCLCPP_WARN(this->get_logger(), "Map change service not available");
    }
  }
  
  /**
   * @brief Handle VDA5050 orders
   * @param msg VDA5050 order message
   * 
   * Processes VDA5050 orders received from a fleet management system:
   * 1. Parses the JSON order
   * 2. Stores order information in the database
   * 3. Visualizes the order for debugging
   */
  void handle_vda_order(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received VDA 5050 order: %s", msg->data.c_str());
    
    // Parse JSON and convert to VDA 5050 order
    // In a real implementation, this would parse the JSON string into the current_order_ object
    
    // For demo purposes, we'll just store the order in the database
    multi_map_nav::vda5050::Order demo_order;
    demo_order.order_id = "demo_order_1";
    demo_order.order_update_id = 1;
    
    // Create some demo nodes
    multi_map_nav::vda5050::Node node1;
    node1.node_id = "node1";
    node1.sequence_id = "1";
    node1.map_id = current_map_;
    node1.released = true;
    node1.actions.push_back("WAITING");
    
    multi_map_nav::vda5050::Node node2;
    node2.node_id = "node2";
    node2.sequence_id = "2";
    node2.map_id = current_map_;
    node2.released = true;
    
    demo_order.nodes.push_back(node1);
    demo_order.nodes.push_back(node2);
    
    // Create a demo edge
    multi_map_nav::vda5050::Edge edge1;
    edge1.edge_id = "edge1";
    edge1.sequence_id = "1";
    edge1.start_node_id = "node1";
    edge1.end_node_id = "node2";
    edge1.map_id = current_map_;
    edge1.released = true;
    
    demo_order.edges.push_back(edge1);
    
    // Store the order and nodes in the database
    db_.store_vda5050_order(demo_order);
    for (const auto& node : demo_order.nodes) {
      db_.add_vda5050_node(node);
    }
    
    // Visualize the order
    visualize_vda_order(demo_order);
  }
  
  /**
   * @brief Publish VDA5050 state updates
   * 
   * Periodically publishes the current state of the robot in VDA5050 format.
   */
  void publish_vda_state()
  {
    // In a real implementation, this would publish the current state in VDA 5050 format
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "{ \"orderId\": \"demo_order_1\", \"orderUpdateId\": 1, \"zoneSetId\": \"default\", "
                "\"lastNodeId\": \"node1\", \"lastNodeSequenceId\": 1, \"driving\": true, "
                "\"paused\": false, \"newBaseRequest\": false, \"distanceSinceLastNode\": 0.0, "
                "\"actionStates\": [], \"errors\": [], \"information\": [] }";
    vda_state_pub_->publish(std::move(msg));
  }
  
  /**
   * @brief Create visualization for VDA5050 orders
   * @param order VDA5050 order to visualize
   * 
   * Creates a path visualization in RViz for debugging VDA5050 orders.
   */
  void visualize_vda_order(const multi_map_nav::vda5050::Order& order)
  {
    // Create a path message for visualization
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = current_map_;
    path_msg.header.stamp = this->now();
    
    for (const auto& node : order.nodes) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose = node.pose;
      path_msg.poses.push_back(pose);
    }
    
    vda_visualization_pub_->publish(path_msg);
  }
  
  /**
   * @brief Plan a path between maps
   * @param start_map Starting map name
   * @param start_pose Starting pose
   * @param goal_map Goal map name
   * @param goal_pose Goal pose
   * @param[out] wormhole_path Path through wormholes
   * @return true if planning succeeded, false otherwise
   * 
   * This function encapsulates the planning logic for multi-map navigation,
   * separating it from the execution logic for better modularity.
   */
  bool plan_multi_map_path(
    const std::string& start_map,
    const geometry_msgs::msg::Pose& start_pose,
    const std::string& goal_map,
    const geometry_msgs::msg::Pose& goal_pose,
    std::vector<multi_map_nav::Wormhole>& wormhole_path)
  {
    if (start_map == goal_map) {
      // No wormholes needed for same-map navigation
      return true;
    }
    
    RCLCPP_INFO(this->get_logger(), "Planning path from %s to %s", 
              start_map.c_str(), goal_map.c_str());
    
    return db_.find_path(start_map, start_pose, goal_map, goal_pose, wormhole_path);
  }

  /**
   * @brief Navigate to a specific position on the current map
   * @param target_pose Target pose
   * @param feedback Feedback message to update
   * @param[in,out] completed_steps Steps completed so far
   * @param total_steps Total steps for the entire navigation
   * @return true if navigation succeeded, false otherwise
   * 
   * This function encapsulates single-map navigation logic, making it
   * reusable for both direct navigation and wormhole approach navigation.
   */
  bool navigate_on_current_map(
    const geometry_msgs::msg::Pose& target_pose,
    std::shared_ptr<MultiMapNavigate::Feedback> feedback,
    float& completed_steps,
    float total_steps,
    const std::shared_ptr<GoalHandleMultiMapNavigate>& goal_handle)
  {
    // Simulate navigation steps
    for (int i = 1; i <= 5 && rclcpp::ok(); ++i) {
      // Check if goal was canceled
      if (goal_handle->is_canceling()) {
        return false;
      }
      
      completed_steps += 1.0f;
      
      // Update feedback
      feedback->percent_complete = (completed_steps / total_steps) * 100.0f;
      feedback->status = "Navigating to position...";
      
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Published feedback: %.1f%% complete", 
                 feedback->percent_complete);
      
      // Simulate work
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    
    return true;
  }
  
  /**
   * @brief Handle new goal requests
   * @param uuid Goal UUID
   * @param goal Goal message
   * @return Action response (accept, reject, etc.)
   */
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MultiMapNavigate::Goal> goal)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Received goal request to navigate to map '%s'",
      goal->map_name.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  
  /**
   * @brief Handle goal cancellation requests
   * @param goal_handle Goal handle
   * @return Cancellation response
   */
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMultiMapNavigate> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  
  /**
   * @brief Handle accepted goals
   * @param goal_handle Goal handle
   * 
   * Starts execution of an accepted navigation goal in a separate thread.
   */
  void handle_accepted(const std::shared_ptr<GoalHandleMultiMapNavigate> goal_handle)
  {
    // Start a new thread to process the goal
    std::thread{std::bind(&MultiMapNavigationServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }
  
  /**
   * @brief Execute a navigation goal
   * @param goal_handle Goal handle
   * 
   * Main execution function for multi-map navigation goals:
   * 1. Determines if map transitions are necessary
   * 2. Finds wormhole paths if required
   * 3. Navigates to wormholes and performs map transitions
   * 4. Provides regular feedback on navigation progress
   * 5. Handles success/failure results
   */
  void execute(const std::shared_ptr<GoalHandleMultiMapNavigate> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    
    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MultiMapNavigate::Feedback>();
    auto result = std::make_shared<MultiMapNavigate::Result>();
    
    // Check if we need to navigate through wormholes
    bool need_wormhole = (current_map_ != goal->map_name);
    
    // Set up initial feedback
    feedback->current_map = current_map_;
    feedback->status = "Starting navigation";
    feedback->percent_complete = 0.0;
    
    // Find the wormhole path if needed
    std::vector<multi_map_nav::Wormhole> wormhole_path;
    
    if (need_wormhole) {
      RCLCPP_INFO(this->get_logger(), "Looking for wormhole path from %s to %s", 
                 current_map_.c_str(), goal->map_name.c_str());
                 
      if (!plan_multi_map_path(current_map_, geometry_msgs::msg::Pose(), 
                               goal->map_name, goal->pose.pose, wormhole_path)) {
        result->success = false;
        result->message = "Could not find a path between maps";
        goal_handle->succeed(result);
        RCLCPP_ERROR(this->get_logger(), "No wormhole path found");
        return;
      }
      
      RCLCPP_INFO(this->get_logger(), "Found wormhole path with %zu transitions", wormhole_path.size());
    }
    
    // Simulate steps of navigation with feedback
    float total_steps = 10.0f; // Base steps
    if (!wormhole_path.empty()) {
      // Add 5 steps per wormhole transition
      total_steps += wormhole_path.size() * 5.0f;
    }
    
    float completed_steps = 0.0f;
    
    // First leg: navigate to first wormhole if needed
    if (!wormhole_path.empty()) {
      if (!navigate_on_current_map(geometry_msgs::msg::Pose(), feedback, completed_steps, total_steps, goal_handle)) {
        result->success = false;
        result->message = "Goal canceled";
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
    }
    
    // Navigate through each wormhole
    for (size_t wh_idx = 0; wh_idx < wormhole_path.size(); ++wh_idx) {
      const auto& wormhole = wormhole_path[wh_idx];
      
      feedback->status = "Transitioning from " + wormhole.source_map + " to " + wormhole.target_map;
      goal_handle->publish_feedback(feedback);
      
      // Simulate map transition
      change_map(wormhole.target_map);
      
      feedback->current_map = wormhole.target_map;
      feedback->status = "Transitioned to " + wormhole.target_map;
      
      completed_steps += 2.0f;
      feedback->percent_complete = (completed_steps / total_steps) * 100.0f;
      goal_handle->publish_feedback(feedback);
      
      // If not the last wormhole, navigate to the next one
      if (wh_idx < wormhole_path.size() - 1) {
        if (!navigate_on_current_map(geometry_msgs::msg::Pose(), feedback, completed_steps, total_steps, goal_handle)) {
          result->success = false;
          result->message = "Goal canceled";
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
          return;
        }
      }
    }
    
    // Final leg: navigate to the goal pose
    if (!navigate_on_current_map(goal->pose.pose, feedback, completed_steps, total_steps, goal_handle)) {
      result->success = false;
      result->message = "Goal canceled";
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }
    
    // Set up result
    if (current_map_ != goal->map_name) {
      change_map(goal->map_name);  // Ensure we're on the target map at the end
    }
    
    result->success = true;
    result->message = "Navigation completed successfully";
    goal_handle->succeed(result);
    
    RCLCPP_INFO(this->get_logger(), "Goal succeeded, now on map: %s", current_map_.c_str());
  }
};

/**
 * @brief Main function
 * @param argc Number of command-line arguments
 * @param argv Command-line arguments
 * @return Exit code
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiMapNavigationServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
