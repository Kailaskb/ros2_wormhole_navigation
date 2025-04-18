#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "multi_map_nav/wormhole_db.hpp"

class MapVisualizer : public rclcpp::Node
{
public:
  MapVisualizer() : Node("map_visualizer")
  {
    // Declare parameters
    this->declare_parameter("db_path", "/tmp/wormhole.db");
    
    // Get parameters
    std::string db_path = this->get_parameter("db_path").as_string();
    
    // Initialize wormhole database
    db_.initialize(db_path);
    
    // Publishers
    markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/multi_map_nav/visualization", 10);
    
    // Timer for periodic visualization updates
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MapVisualizer::publish_visualization, this));
    
    RCLCPP_INFO(this->get_logger(), "Map visualizer started");
  }

private:
  multi_map_nav::WormholeDB db_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  void publish_visualization()
  {
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Get all wormholes from the database
    // We'll query for wormholes from all maps
    std::vector<std::string> maps = {"map1", "map2", "map3"};  // Example map list
    
    int marker_id = 0;
    
    for (const auto& map_name : maps) {
      std::vector<multi_map_nav::Wormhole> wormholes;
      if (db_.get_wormholes(map_name, wormholes)) {
        for (const auto& wormhole : wormholes) {
          // Create marker for source pose
          visualization_msgs::msg::Marker source_marker;
          source_marker.header.frame_id = "map";  // Assuming a global frame
          source_marker.header.stamp = this->now();
          source_marker.ns = "wormhole_source";
          source_marker.id = marker_id++;
          source_marker.type = visualization_msgs::msg::Marker::ARROW;
          source_marker.action = visualization_msgs::msg::Marker::ADD;
          source_marker.pose = wormhole.source_pose;
          source_marker.scale.x = 0.8;  // Arrow length
          source_marker.scale.y = 0.2;  // Arrow width
          source_marker.scale.z = 0.2;  // Arrow height
          source_marker.color.r = 1.0;
          source_marker.color.g = 0.0;
          source_marker.color.b = 0.0;
          source_marker.color.a = 1.0;
          marker_array.markers.push_back(source_marker);
          
          // Create marker for target pose
          visualization_msgs::msg::Marker target_marker;
          target_marker.header.frame_id = "map";  // Assuming a global frame
          target_marker.header.stamp = this->now();
          target_marker.ns = "wormhole_target";
          target_marker.id = marker_id++;
          target_marker.type = visualization_msgs::msg::Marker::ARROW;
          target_marker.action = visualization_msgs::msg::Marker::ADD;
          target_marker.pose = wormhole.target_pose;
          target_marker.scale.x = 0.8;  // Arrow length
          target_marker.scale.y = 0.2;  // Arrow width
          target_marker.scale.z = 0.2;  // Arrow height
          target_marker.color.r = 0.0;
          target_marker.color.g = 1.0;
          target_marker.color.b = 0.0;
          target_marker.color.a = 1.0;
          marker_array.markers.push_back(target_marker);
          
          // Create line connecting source and target
          visualization_msgs::msg::Marker connection_marker;
          connection_marker.header.frame_id = "map";  // Assuming a global frame
          connection_marker.header.stamp = this->now();
          connection_marker.ns = "wormhole_connection";
          connection_marker.id = marker_id++;
          connection_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
          connection_marker.action = visualization_msgs::msg::Marker::ADD;
          
          geometry_msgs::msg::Point p1;
          p1.x = wormhole.source_pose.position.x;
          p1.y = wormhole.source_pose.position.y;
          p1.z = wormhole.source_pose.position.z;
          
          geometry_msgs::msg::Point p2;
          p2.x = wormhole.target_pose.position.x;
          p2.y = wormhole.target_pose.position.y;
          p2.z = wormhole.target_pose.position.z;
          
          connection_marker.points.push_back(p1);
          connection_marker.points.push_back(p2);
          
          connection_marker.scale.x = 0.1;  // Line width
          connection_marker.color.r = 1.0;
          connection_marker.color.g = 1.0;
          connection_marker.color.b = 0.0;
          connection_marker.color.a = 0.8;
          marker_array.markers.push_back(connection_marker);
          
          // Add text marker for map name at source
          visualization_msgs::msg::Marker source_text;
          source_text.header.frame_id = "map";
          source_text.header.stamp = this->now();
          source_text.ns = "wormhole_text";
          source_text.id = marker_id++;
          source_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
          source_text.action = visualization_msgs::msg::Marker::ADD;
          source_text.pose = wormhole.source_pose;
          source_text.pose.position.z += 0.5;  // Offset text above the arrow
          source_text.text = wormhole.source_map;
          source_text.scale.z = 0.3;  // Text height
          source_text.color.r = 1.0;
          source_text.color.g = 1.0;
          source_text.color.b = 1.0;
          source_text.color.a = 1.0;
          marker_array.markers.push_back(source_text);
          
          // Add text marker for map name at target
          visualization_msgs::msg::Marker target_text;
          target_text.header.frame_id = "map";
          target_text.header.stamp = this->now();
          target_text.ns = "wormhole_text";
          target_text.id = marker_id++;
          target_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
          target_text.action = visualization_msgs::msg::Marker::ADD;
          target_text.pose = wormhole.target_pose;
          target_text.pose.position.z += 0.5;  // Offset text above the arrow
          target_text.text = wormhole.target_map;
          target_text.scale.z = 0.3;  // Text height
          target_text.color.r = 1.0;
          target_text.color.g = 1.0;
          target_text.color.b = 1.0;
          target_text.color.a = 1.0;
          marker_array.markers.push_back(target_text);
        }
      }
    }
    
    // Publish markers
    markers_pub_->publish(marker_array);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapVisualizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
