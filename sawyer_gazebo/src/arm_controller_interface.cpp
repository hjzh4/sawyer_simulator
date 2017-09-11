/***************************************************************************
* Copyright (c) 2013-2017, Rethink Robotics Inc.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
**************************************************************************/

#include <sawyer_gazebo/arm_controller_interface.h>
#include <controller_manager_msgs/SwitchController.h>

namespace sawyer_gazebo {

void ArmControllerInterface::init(ros::NodeHandle& nh, std::string side,
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager) {
  current_mode_ = -1;
  side_ = side;
  controller_manager_ = controller_manager;
  joint_command_timeout_sub_ = nh.subscribe("limb/"+side_+"/joint_command_timeout", 1,
                       &ArmControllerInterface::jointCommandTimeoutCallback, this);
  joint_command_sub_ = nh.subscribe("limb/"+side_+"/joint_command", 1,
                       &ArmControllerInterface::jointCommandCallback, this);
  joint_state_sub_ = nh.subscribe("limb/"+side_+"/gravity_compensation_torques", 1,
                       &ArmControllerInterface::jointStateCallback, this);
  jnt_cmd_pub_ = nh.advertise<intera_core_msgs::JointCommand>("limb/"+side_+"/joint_command", 1);
  // Update at 100Hz
  auto command_timeout = std::make_shared<ros::Duration>(0.2);
  usr_command_ = true;
  command_timeout_buffer_.set(command_timeout);
  last_command_time_ = ros::Time::now();
  update_timer_ = nh.createTimer(100, &ArmControllerInterface::update, this);
}

void ArmControllerInterface::jointStateCallback(const intera_core_msgs::SEAJointStateConstPtr& msg)
{
  auto joint_state = std::make_shared<intera_core_msgs::SEAJointState>(*msg);
  joint_state_buffer_.set(joint_state);  // FIXME - what happens when different joint states messages are received?
}

void ArmControllerInterface::update(const ros::TimerEvent& e)
{
  std::shared_ptr<const ros::Duration> command_timeout;
  command_timeout_buffer_.get(command_timeout);
  if (usr_command_ &&
      command_timeout.get() &&
      (ros::Time::now() - last_command_time_ > *command_timeout.get()))
  {
    ROS_INFO_STREAM_NAMED("sawyer_control_plugin", "Joint command timeout reached: " << *command_timeout.get());
    publishCmdCurrentPose(usr_command_);
    usr_command_ = false;
  }
}

void ArmControllerInterface::jointCommandTimeoutCallback(const std_msgs::Float64 msg) {
  ROS_DEBUG_STREAM_NAMED("sawyer_control_plugin", "Joint command timeout: " << msg.data);
  auto command_timeout = std::make_shared<ros::Duration>(msg.data);
  command_timeout_buffer_.set(command_timeout);
}

std::string ArmControllerInterface::getControllerString(std::string mode_str){
    std::ostringstream ss;
    ss << side_ <<"_joint_"<<mode_str<<"_controller";
    return ss.str();
}

void ArmControllerInterface::publishCmdCurrentPose(bool usr_cmd)
{
  std::shared_ptr<const intera_core_msgs::SEAJointState> joint_state;
  joint_state_buffer_.get(joint_state);
  if (joint_state.get())
  {
    if(usr_cmd){
      intera_core_msgs::JointCommand jnt_cmd;
      jnt_cmd.mode = intera_core_msgs::JointCommand::POSITION_MODE;
      auto j_state = *joint_state.get();
      jnt_cmd.names = j_state.name;
      jnt_cmd.position = j_state.actual_position;
      jnt_cmd.header.stamp = ros::Time::now();
      jnt_cmd.header.seq = std::numeric_limits<unsigned int>::max();
      last_jnt_cmd_ = jnt_cmd;
    }
    jnt_cmd_pub_.publish(last_jnt_cmd_);
  }
}

void ArmControllerInterface::jointCommandCallback(const intera_core_msgs::JointCommandConstPtr& msg) {
  if(msg->header.seq < std::numeric_limits<unsigned int>::max()) //
  {
    last_command_time_ = ros::Time::now();
    usr_command_ = true;
  }
  std::vector<std::string> start_controllers;
  std::vector<std::string> stop_controllers;
  if(current_mode_ != msg->mode)
  {
    switch (msg->mode)
    {
      case intera_core_msgs::JointCommand::POSITION_MODE:
      case intera_core_msgs::JointCommand::TRAJECTORY_MODE:
        start_controllers.push_back(getControllerString("position"));
        start_controllers.push_back(getControllerString("gravity"));
        stop_controllers.push_back(getControllerString("velocity"));
        stop_controllers.push_back(getControllerString("effort"));
        break;
      case intera_core_msgs::JointCommand::VELOCITY_MODE:
        start_controllers.push_back(getControllerString("velocity"));
        start_controllers.push_back(getControllerString("gravity"));
        stop_controllers.push_back(getControllerString("position"));
        stop_controllers.push_back(getControllerString("effort"));
        break;
      case intera_core_msgs::JointCommand::TORQUE_MODE:
        start_controllers.push_back(getControllerString("effort"));
        start_controllers.push_back(getControllerString("gravity"));
        stop_controllers.push_back(getControllerString("position"));
        stop_controllers.push_back(getControllerString("velocity"));
        break;
      default:
        ROS_ERROR_STREAM_NAMED("ros_control_plugin", "Unknown command mode " << msg->mode);
        return;
    }
    if (!controller_manager_->switchController(start_controllers, stop_controllers,
                              controller_manager_msgs::SwitchController::Request::BEST_EFFORT))
    {
      ROS_ERROR_STREAM_NAMED("ros_control_plugin", "Failed to switch controllers");
    }
    else
    {
      current_mode_ = msg->mode;
      //is_disabled = false;
      //ROS_INFO_NAMED("ros_control_plugin", "Robot is enabled");
      ROS_INFO_STREAM_NAMED("ros_control_plugin", "Controller " << start_controllers[0]
                            << " started and " << stop_controllers[0] + " and " + stop_controllers[1] << " stopped.");
      //ROS_INFO_NAMED("ros_control_plugin", "Gravity compensation was turned on by mode command callback");
    }
  }

}
}
