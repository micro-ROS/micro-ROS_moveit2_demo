/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*      Title     : servo_cpp_interface_demo.cpp
 *      Project   : moveit_servo
 *      Created   : 07/13/2020
 *      Author    : Adam Pettinger
 */

// ROS
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>

// Servo
#include <moveit_servo/servo_parameters.cpp>
#include <moveit_servo/servo.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.servo_demo_node.cpp");

class ServoCppDemo
{
public:
  ServoCppDemo(rclcpp::Node::SharedPtr node) : node_(node), movement_mode_jog_(false), count_(0)
  {
    // Create the planning_scene_monitor
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        node_, "robot_description", tf_buffer_, "planning_scene_monitor");
    planning_scene_monitor_->providePlanningSceneService();

    // Get the planning_scene_monitor to publish scene diff's for RViz visualization
    if (planning_scene_monitor_->getPlanningScene())
    {
      planning_scene_monitor_->startStateMonitor("/joint_states");
      planning_scene_monitor_->setPlanningScenePublishingFrequency(25);
      planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                            "/moveit_servo/publish_planning_scene");
      planning_scene_monitor_->startSceneMonitor();
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "Planning scene not configured");
    }

    // We need 2 different publishers for the different command types
    joint_cmd_pub_ = node_->create_publisher<control_msgs::msg::JointJog>("servo_server/delta_joint_cmds", 10);
    twist_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("servo_server/delta_twist_cmds", 10);

    scene_pub = node->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);

    auto servo_parameters = std::make_shared<moveit_servo::ServoParameters>();
    if (!moveit_servo::readParameters(servo_parameters, node_, LOGGER))
    {
      RCLCPP_ERROR(LOGGER, "Could not get parameters");
    }

    // Create Servo and start it
    servo_ = std::make_unique<moveit_servo::Servo>(node_, servo_parameters, planning_scene_monitor_);
    servo_->start();
    
    // timer_ = node_->create_wall_timer(50ms, std::bind(&ServoCppDemo::publishCommands, this));
  }

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr scene_pub;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_pub_;

  bool  movement_mode_jog_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
  std::unique_ptr<moveit_servo::Servo> servo_;

  size_t count_;
  rclcpp::TimerBase::SharedPtr timer_;

  void publishCommands()
  {
    // Publish joint commands for a little bit
    if (count_ < 100)
    {
      auto msg = std::make_unique<control_msgs::msg::JointJog>();
      msg->header.stamp = node_->now();
      msg->joint_names.push_back("joint1");
      msg->velocities.push_back(0.1);
      joint_cmd_pub_->publish(std::move(msg));
      ++count_;
    }
    // Then switch to twist commands
    else if (count_ < 200)
    {
      auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
      msg->header.stamp = node_->now();
      msg->header.frame_id = "link5";
      msg->twist.linear.x = 0.1;
      msg->twist.linear.z = 0.1;
      twist_cmd_pub_->publish(std::move(msg));
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.use_intra_process_comms(true);
  auto node = std::make_shared<rclcpp::Node>("servo_demo_node", node_options);

  // Pause for RViz to come up..
  rclcpp::sleep_for(std::chrono::seconds(4));

  // We need to initialize the planning_scene_monitor here before publishing the collision object
  ServoCppDemo demo(node);

  // Spin
  auto executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  executor->spin();

  while(1){}

  rclcpp::shutdown();
  return 0;
}
