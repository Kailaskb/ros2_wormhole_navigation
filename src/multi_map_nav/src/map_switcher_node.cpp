/**
 * @file map_switcher_node.cpp
 * @brief Implementation of the map switching node for multi-map navigation
 * 
 * This node is responsible for handling map switching during multi-map navigation.
 * It subscribes to different map topics and a current map topic, then publishes
 * the active map based on the current map name. This ensures that the navigation
 * stack always has the correct map available.
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

/**
 * @class MapSwitcherNode
 * @brief Node that manages map switching for multi-map navigation
 * 
 * The MapSwitcherNode subscribes to multiple map topics (one for each room map)
 * and publishes the appropriate map based on the current map name received on
 * the /multi_map_nav/current_map topic. If no valid map is available, it can
 * create a placeholder map to ensure the navigation system has a map to work with.
 */
class MapSwitcherNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for MapSwitcherNode
   * 
   * Initializes publishers, subscribers, and sets up the periodic map publishing timer.
   */
  MapSwitcherNode() : Node("map_switcher")
  {
    // Create publishers for the active map
    active_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
    
    // Subscribe to current map updates
    map_name_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/multi_map_nav/current_map", 10,
      std::bind(&MapSwitcherNode::handle_map_change, this, std::placeholders::_1));
    
    // Subscribe to all map topics
    room1_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map_room1", 1, 
      std::bind(&MapSwitcherNode::handle_room1_map, this, std::placeholders::_1));
      
    room2_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map_room2", 1, 
      std::bind(&MapSwitcherNode::handle_room2_map, this, std::placeholders::_1));
      
    room3_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map_room3", 1, 
      std::bind(&MapSwitcherNode::handle_room3_map, this, std::placeholders::_1));
    
    // Default map
    current_map_name_ = "room1";
    
    // Add a periodic publisher to ensure map gets published regularly
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MapSwitcherNode::publish_current_map, this));
    
    RCLCPP_INFO(this->get_logger(), "Map switcher initialized with default map: %s", 
               current_map_name_.c_str());
  }

private:
  // Publishers and subscribers
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr active_map_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_name_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr room1_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr room2_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr room3_map_sub_;
  
  // Map storage
  nav_msgs::msg::OccupancyGrid room1_map_;
  nav_msgs::msg::OccupancyGrid room2_map_;
  nav_msgs::msg::OccupancyGrid room3_map_;
  nav_msgs::msg::OccupancyGrid last_published_map_;
  
  std::string current_map_name_;
  
  /**
   * @brief Callback for current map name updates
   * @param msg Message containing the current map name
   */
  void handle_map_change(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string new_map_name = msg->data;
    
    if (new_map_name != current_map_name_) {
      RCLCPP_INFO(this->get_logger(), "Switching map from %s to %s", 
                 current_map_name_.c_str(), new_map_name.c_str());
      current_map_name_ = new_map_name;
      
      // Publish the appropriate map
      publish_current_map();
    }
  }
  
  /**
   * @brief Callback for room1 map updates
   * @param msg Message containing the room1 map data
   */
  void handle_room1_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received room1 map with size %zu", msg->data.size());
    room1_map_ = *msg;
    if (current_map_name_ == "room1") {
      publish_current_map();
    }
  }
  
  /**
   * @brief Callback for room2 map updates
   * @param msg Message containing the room2 map data
   */
  void handle_room2_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received room2 map with size %zu", msg->data.size());
    room2_map_ = *msg;
    if (current_map_name_ == "room2") {
      publish_current_map();
    }
  }
  
  /**
   * @brief Callback for room3 map updates
   * @param msg Message containing the room3 map data
   */
  void handle_room3_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received room3 map with size %zu", msg->data.size());
    room3_map_ = *msg;
    if (current_map_name_ == "room3") {
      publish_current_map();
    }
  }
  
  /**
   * @brief Create a placeholder map if no real map is available
   * 
   * This creates a simple empty map with walls around the perimeter,
   * ensuring that the navigation system has a valid map to work with
   * even if the actual map data hasn't been received yet.
   */
  void create_placeholder_map()
  {
    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id = "map";
    map.header.stamp = this->now();
    
    map.info.resolution = 0.05;  // 5cm per pixel
    map.info.width = 200;
    map.info.height = 200;
    map.info.origin.position.x = -5.0;
    map.info.origin.position.y = -5.0;
    map.info.origin.orientation.w = 1.0;
    
    // Create a simple map with borders
    std::vector<int8_t> map_data(map.info.width * map.info.height, 0);
    
    // Add walls
    // Top and bottom walls
    for (unsigned int i = 0; i < map.info.width; ++i) {
      map_data[i] = 100;  // Top wall
      map_data[(map.info.height - 1) * map.info.width + i] = 100;  // Bottom wall
    }
    
    // Left and right walls
    for (unsigned int i = 0; i < map.info.height; ++i) {
      map_data[i * map.info.width] = 100;  // Left wall
      map_data[i * map.info.width + map.info.width - 1] = 100;  // Right wall
    }
    
    map.data = map_data;
    last_published_map_ = map;
  }
  
  /**
   * @brief Publish the current map based on the current_map_name_
   * 
   * This function is called periodically and whenever the current map
   * changes. It publishes the appropriate map based on the current_map_name_
   * value. If the requested map isn't available, it can publish a
   * placeholder map instead.
   */
  void publish_current_map()
  {
    nav_msgs::msg::OccupancyGrid map_to_publish;
    
    if (current_map_name_ == "room1") {
      map_to_publish = room1_map_;
    } else if (current_map_name_ == "room2") {
      map_to_publish = room2_map_;
    } else if (current_map_name_ == "room3") {
      map_to_publish = room3_map_;
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown map name: %s", current_map_name_.c_str());
      return;
    }
    
    // Only publish if we have valid data
    if (map_to_publish.data.size() > 0) {
      // Make sure the frame_id is correctly set to "map"
      map_to_publish.header.frame_id = "map";
      map_to_publish.header.stamp = this->now();
      
      active_map_pub_->publish(map_to_publish);
      RCLCPP_INFO(this->get_logger(), "Published map: %s with size %zu", 
                  current_map_name_.c_str(), map_to_publish.data.size());
    } else {
      RCLCPP_ERROR(this->get_logger(), "No map data available for: %s, size: %zu", 
                   current_map_name_.c_str(), map_to_publish.data.size());
      
      // Debug information about which maps are loaded
      RCLCPP_INFO(this->get_logger(), "Map sizes - room1: %zu, room2: %zu, room3: %zu",
                  room1_map_.data.size(), room2_map_.data.size(), room3_map_.data.size());
      
      // Create a placeholder map if no data is available
      if (last_published_map_.data.size() == 0) {
        RCLCPP_WARN(this->get_logger(), "Creating placeholder map...");
        create_placeholder_map();
      }
      
      // Publish the last valid map or placeholder
      if (last_published_map_.data.size() > 0) {
        last_published_map_.header.stamp = this->now();
        active_map_pub_->publish(last_published_map_);
        RCLCPP_INFO(this->get_logger(), "Published placeholder map");
      }
    }
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
  auto node = std::make_shared<MapSwitcherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
