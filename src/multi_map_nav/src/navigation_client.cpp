#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "multi_map_nav/action/multi_map_navigate.hpp"

using namespace std::chrono_literals;

class MultiMapNavigationClient : public rclcpp::Node
{
public:
  using MultiMapNavigate = multi_map_nav::action::MultiMapNavigate;
  using GoalHandleMultiMapNavigate = rclcpp_action::ClientGoalHandle<MultiMapNavigate>;
  
  MultiMapNavigationClient()
  : Node("multi_map_navigation_client")
  {
    // Parameters for navigation goal
    this->declare_parameter("map_name", "room2");
    this->declare_parameter("x", 2.0);
    this->declare_parameter("y", 3.0);
    this->declare_parameter("z", 0.0);
    this->declare_parameter("qx", 0.0);
    this->declare_parameter("qy", 0.0);
    this->declare_parameter("qz", 0.0);
    this->declare_parameter("qw", 1.0);
    
    // Create the action client
    client_ = rclcpp_action::create_client<MultiMapNavigate>(
      this, "multi_map_navigate");
    
    // Send the goal after a brief delay to allow connections to establish
    timer_ = this->create_wall_timer(
      2s, std::bind(&MultiMapNavigationClient::send_goal, this));
  }

private:
  rclcpp_action::Client<MultiMapNavigate>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  void send_goal()
  {
    // Cancel the timer to prevent sending multiple goals
    timer_->cancel();
    
    // Check if server is available
    if (!client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }
    
    // Get parameters for the goal
    std::string map_name = this->get_parameter("map_name").as_string();
    double x = this->get_parameter("x").as_double();
    double y = this->get_parameter("y").as_double();
    double z = this->get_parameter("z").as_double();
    double qx = this->get_parameter("qx").as_double();
    double qy = this->get_parameter("qy").as_double();
    double qz = this->get_parameter("qz").as_double();
    double qw = this->get_parameter("qw").as_double();
    
    // Create a goal
    auto goal_msg = MultiMapNavigate::Goal();
    goal_msg.map_name = map_name;
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.position.z = z;
    goal_msg.pose.pose.orientation.x = qx;
    goal_msg.pose.pose.orientation.y = qy;
    goal_msg.pose.pose.orientation.z = qz;
    goal_msg.pose.pose.orientation.w = qw;
    
    RCLCPP_INFO(this->get_logger(), 
      "Sending goal to navigate to map '%s' at position (%f, %f, %f)",
      map_name.c_str(), x, y, z);
    
    // Send the goal
    auto send_goal_options = rclcpp_action::Client<MultiMapNavigate>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&MultiMapNavigationClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(&MultiMapNavigationClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&MultiMapNavigationClient::result_callback, this, std::placeholders::_1);
    
    client_->async_send_goal(goal_msg, send_goal_options);
  }
  
  void goal_response_callback(const GoalHandleMultiMapNavigate::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }
  
  void feedback_callback(
    GoalHandleMultiMapNavigate::SharedPtr,
    const std::shared_ptr<const MultiMapNavigate::Feedback> feedback)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Received feedback: %.1f%% complete, current map: %s, status: %s",
      feedback->percent_complete, feedback->current_map.c_str(), feedback->status.c_str());
  }
  
  void result_callback(const GoalHandleMultiMapNavigate::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal succeeded: %s", result.result->message.c_str());
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
    rclcpp::shutdown();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiMapNavigationClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
