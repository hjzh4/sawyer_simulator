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

#ifndef SAWYER_POSITION_CONTROLLER_H
#define SAWYER_POSITION_CONTROLLER_H

#include <sawyer_sim_controllers/joint_group_position_controller.h>
#include <intera_core_msgs/JointCommand.h>
#include <ros/node_handle.h>

#include <control_toolbox/pid.h>

namespace sawyer_sim_controllers
{
  class SawyerPositionController : public sawyer_sim_controllers::JointGroupPositionController
  {
  public:
    virtual ~SawyerPositionController() {sub_joint_command_.shutdown();}
    virtual bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);

  private:
    ros::Subscriber sub_joint_command_;

    void jointCommandCB(const intera_core_msgs::JointCommandConstPtr& msg);
  };
}

#endif