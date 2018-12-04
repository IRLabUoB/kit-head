/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  Copyright (c) 2013, The Johns Hopkins University
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
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
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

/* Author: Dave Coleman, Jonathan Bohren
   Desc:   Hardware Interface for any simulated robot in Gazebo
*/


#include <kit_head_hw/default_kit_head_hw_sim.h>

namespace kit_head_hw
{


bool DefaultKITHeadHWSim::initSim(
  const std::string& robot_namespace,
  ros::NodeHandle model_nh,
  gazebo::physics::ModelPtr parent_model,
  const urdf::Model *const urdf_model,
  std::vector<transmission_interface::TransmissionInfo> transmissions)
  {

    // filter transmission interface

    std::vector<transmission_interface::TransmissionInfo> transmissions_filtered;
    int njoints = 7;

    std::vector<std::string> joint_names;

    joint_names.push_back( robot_namespace + std::string("_neck_pitch_joint") );
    joint_names.push_back( robot_namespace + std::string("_neck_roll_joint") );
    joint_names.push_back( robot_namespace + std::string("_neck_yaw_joint") );
    joint_names.push_back( robot_namespace + std::string("_head_tilt_joint") );
    joint_names.push_back( robot_namespace + std::string("_eyes_tilt_joint") );
    joint_names.push_back( robot_namespace + std::string("_left_eye_joint") );
    joint_names.push_back( robot_namespace + std::string("_right_eye_joint") );

    for (int j = 0; j < njoints; ++j)
    {
      // std::cout << "Check joint " << joint_names_[j] << std::endl;
      std::vector<transmission_interface::TransmissionInfo>::iterator it = transmissions.begin();
      for(; it != transmissions.end(); ++it)
      {
        std::cout << "With transmission " << it->name_ << " trying to match: " << joint_names[j] << std::endl;
        if (joint_names[j].compare(it->joints_[0].name_) == 0)
        {
          transmissions_filtered.push_back( *it );
          std::cout << "Found a match for transmission " << it->name_ << std::endl;
        }
      }
    }

    bool ret = gazebo_ros_control::DefaultRobotHWSim::initSim(robot_namespace, model_nh, parent_model, urdf_model, transmissions_filtered);

    return ret;
  }

void DefaultKITHeadHWSim::writeSim(ros::Time time, ros::Duration period)
{
  gazebo_ros_control::DefaultRobotHWSim::writeSim(time, period);
}
}

PLUGINLIB_EXPORT_CLASS(kit_head_hw::DefaultKITHeadHWSim, gazebo_ros_control::RobotHWSim)