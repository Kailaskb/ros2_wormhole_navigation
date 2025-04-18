#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class RobotStatePublisherNode : public rclcpp::Node
{
public:
  RobotStatePublisherNode() : Node("robot_state_publisher_node")
  {
    // Publisher for robot description
    // ROS2 Humble doesn't support the latched topic parameter directly
    rclcpp::QoS qos(10);
    qos.transient_local();  // This makes it work like a latched topic
    robot_description_publisher_ = this->create_publisher<std_msgs::msg::String>(
      "robot_description", qos);

    // Publish the robot description once on startup
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&RobotStatePublisherNode::publish_robot_description, this));

    RCLCPP_INFO(this->get_logger(), "Robot state publisher initialized");
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_description_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  void publish_robot_description()
  {
    // Cancel the timer so we only publish once
    timer_->cancel();

    // Simple URDF for a differential drive robot
    std::string robot_description = R"(
      <?xml version="1.0"?>
      <robot name="turtlebot3_burger">
        <link name="base_footprint"/>

        <link name="base_link">
          <visual>
            <origin xyz="0 0 0.0" rpy="0 0 0"/>
            <geometry>
              <box size="0.140 0.140 0.143"/>
            </geometry>
            <material name="orange"/>
          </visual>
          <collision>
            <origin xyz="0 0 0.0" rpy="0 0 0"/>
            <geometry>
              <box size="0.140 0.140 0.143"/>
            </geometry>
          </collision>
        </link>

        <link name="wheel_left_link">
          <visual>
            <origin xyz="0 0 0" rpy="1.57 0 0"/>
            <geometry>
              <cylinder length="0.018" radius="0.033"/>
            </geometry>
            <material name="dark"/>
          </visual>
          <collision>
            <origin xyz="0 0 0" rpy="1.57 0 0"/>
            <geometry>
              <cylinder length="0.018" radius="0.033"/>
            </geometry>
          </collision>
        </link>

        <link name="wheel_right_link">
          <visual>
            <origin xyz="0 0 0" rpy="1.57 0 0"/>
            <geometry>
              <cylinder length="0.018" radius="0.033"/>
            </geometry>
            <material name="dark"/>
          </visual>
          <collision>
            <origin xyz="0 0 0" rpy="1.57 0 0"/>
            <geometry>
              <cylinder length="0.018" radius="0.033"/>
            </geometry>
          </collision>
        </link>

        <joint name="base_joint" type="fixed">
          <parent link="base_footprint"/>
          <child link="base_link"/>
          <origin xyz="0 0 0.0715" rpy="0 0 0"/>
        </joint>

        <joint name="wheel_left_joint" type="continuous">
          <parent link="base_link"/>
          <child link="wheel_left_link"/>
          <origin xyz="0 0.08 -0.023" rpy="0 0 0"/>
          <axis xyz="0 1 0"/>
        </joint>

        <joint name="wheel_right_joint" type="continuous">
          <parent link="base_link"/>
          <child link="wheel_right_link"/>
          <origin xyz="0 -0.08 -0.023" rpy="0 0 0"/>
          <axis xyz="0 1 0"/>
        </joint>

        <material name="dark">
          <color rgba="0.2 0.2 0.2 1.0"/>
        </material>

        <material name="orange">
          <color rgba="1.0 0.5 0.2 1.0"/>
        </material>
      </robot>
    )";

    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = robot_description;
    robot_description_publisher_->publish(std::move(msg));
    RCLCPP_INFO(this->get_logger(), "Published robot description");
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotStatePublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
