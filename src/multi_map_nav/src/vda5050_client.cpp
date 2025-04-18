#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class VDA5050Client : public rclcpp::Node
{
public:
  VDA5050Client() : Node("vda5050_client")
  {
    // Publisher for VDA 5050 orders
    order_pub_ = this->create_publisher<std_msgs::msg::String>("/vda5050/order", 10);
    
    // Subscriber for VDA 5050 state
    state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/vda5050/state", 10,
      std::bind(&VDA5050Client::handle_state, this, std::placeholders::_1));
    
    // Send a test order on startup
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&VDA5050Client::send_test_order, this));
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr order_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  void handle_state(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received VDA 5050 state: %s", msg->data.c_str());
    
    try {
      auto state = json::parse(msg->data);
      // Process state if needed
    } catch (const json::parse_error& e) {
      RCLCPP_ERROR(this->get_logger(), "Error parsing state JSON: %s", e.what());
    }
  }
  
  void send_test_order()
  {
    // Only send the order once
    timer_->cancel();
    
    json order;
    order["orderId"] = "test_order_123";
    order["orderUpdateId"] = 1;
    order["zoneSetId"] = "default";
    
    // Create nodes array
    json nodes = json::array();
    
    // Node 1
    json node1;
    node1["nodeId"] = "start";
    node1["sequenceId"] = "0";
    node1["nodePosition"]["x"] = 1.0;
    node1["nodePosition"]["y"] = 1.0;
    node1["nodePosition"]["mapId"] = "map1";
    node1["released"] = true;
    nodes.push_back(node1);
    
    // Node 2
    json node2;
    node2["nodeId"] = "goal";
    node2["sequenceId"] = "1";
    node2["nodePosition"]["x"] = 5.0;
    node2["nodePosition"]["y"] = 5.0;
    node2["nodePosition"]["mapId"] = "map2";
    node2["released"] = true;
    
    // Add actions to node 2
    json actions = json::array();
    json action;
    action["actionType"] = "waiting";
    action["actionId"] = "wait_1";
    action["blockingType"] = "NONE";
    actions.push_back(action);
    node2["actions"] = actions;
    
    nodes.push_back(node2);
    order["nodes"] = nodes;
    
    // Create edges array
    json edges = json::array();
    json edge;
    edge["edgeId"] = "edge_1";
    edge["sequenceId"] = "0";
    edge["startNodeId"] = "start";
    edge["endNodeId"] = "goal";
    edge["released"] = true;
    edges.push_back(edge);
    order["edges"] = edges;
    
    // Publish the order
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = order.dump(2);  // 2 = indentation for pretty printing
    RCLCPP_INFO(this->get_logger(), "Sending VDA 5050 order");
    order_pub_->publish(std::move(msg));
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VDA5050Client>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
