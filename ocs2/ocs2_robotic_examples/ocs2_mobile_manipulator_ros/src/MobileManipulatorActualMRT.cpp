/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_mobile_manipulator_ros/MobileManipulatorActualMRT.h>

#include <ocs2_mobile_manipulator_ros/MobileManipulatorDummyVisualization.h>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <ros/init.h>
#include <ros/package.h>

using namespace ocs2;
using namespace mobile_manipulator;

ActualMRT::ActualMRT(ros::NodeHandle &n) : n_(n)
{
  sub_base_state_ = n_.subscribe("/lio_sam/mapping/odometry", 1, 
                                &ActualMRT::base_odometry_callback, this, ros::TransportHints().tcpNoDelay());
                              
  // TODO the corresponding publisher on the joint controller
  sub_arm_state_ = n_.subscribe("/ur5e_joint_velocity_controller/joint_states", 1, 
                               &ActualMRT::arm_joint_state_callback, this, ros::TransportHints().tcpNoDelay());

  sub_policy_ = n_.subscribe("/mobile_manipulator_mpc_policy", 1, 
                            &ActualMRT::mpc_policy_callback, this, ros::TransportHints().tcpNoDelay());

  pub_base_command_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1); 

  pub_arm_command_ = n_.advertise<geometry_msgs::Twist>("/now_joint_command", 1); 

  pub_observation_ = n_.advertise<ocs2_msgs::mpc_observation>("/mobile_manipulator_mpc_observation", 1); 

  mpcResetServiceClient_ = n_.serviceClient<ocs2_msgs::reset>("/mobile_manipulator_mpc_reset");

}

void ActualMRT::base_odometry_callback(const nav_msgs::Odometry& slam_odom)
{
  base_map_translation_ << slam_odom.pose.pose.position.x,
                             slam_odom.pose.pose.position.y,
                             slam_odom.pose.pose.position.z;
  base_map_orientation_.coeffs() <<  slam_odom.pose.pose.orientation.x,
                                       slam_odom.pose.pose.orientation.y,
                                       slam_odom.pose.pose.orientation.z,
                                       slam_odom.pose.pose.orientation.w;
  Eigen::Vector3d eulerAngle=base_map_orientation_.matrix().eulerAngles(0,1,2);
  theta_ = eulerAngle(2);
}

void ActualMRT::arm_joint_state_callback(const geometry_msgs::Twist& arm_joint)
{
  arm_joint_state_ << arm_joint.linear.x,
                     arm_joint.linear.y,
                     arm_joint.linear.z,
                     arm_joint.angular.x,
                      arm_joint.angular.y,
                     arm_joint.angular.z;
    // std::cout << "msgs recieved in the callback function"  << arm_joint_state_<< std::endl;
}

void ActualMRT::mpc_policy_callback(const ocs2_msgs::mpc_flattened_controller& msg) {
  // read new policy and command from msg
  base_command_.linear.x = msg.data[0].data[0]/100;
  base_command_.angular.z = msg.data[0].data[1]/100;
  arm_command_.linear.x = msg.data[0].data[2]/100;
  arm_command_.linear.y = msg.data[0].data[3]/100;
  arm_command_.linear.z = msg.data[0].data[4]/100;
  arm_command_.angular.x = msg.data[0].data[5]/100;
  arm_command_.angular.y = msg.data[0].data[6]/100;
  arm_command_.angular.z = msg.data[0].data[7]/100;
  // base_command_.linear.x  = 0;
  // base_command_.angular.z = 0;
  // arm_command_.linear.x   = 0;
  // arm_command_.linear.y   = 0;
  // arm_command_.linear.z   = 0;
  // arm_command_.angular.x  = 0;
  // arm_command_.angular.y  = 0;
  // arm_command_.angular.z  = 0;
  std::cout << "the policy received: " << msg.data[0] << std::endl;
}

void ActualMRT::fusion(SystemObservation& fused_observation)
{
  fused_observation.time = ros::Time::now().toSec();
  fused_observation.state.setZero(9);
  fused_observation.input.setZero(8);

  fused_observation.state(0) = base_map_translation_(0);
  fused_observation.state(1) = base_map_translation_(1);
  fused_observation.state(2) = theta_;
  for(int i = 0; i < 6; i++)
  {
    fused_observation.state(i+3) = arm_joint_state_(i);
  }
  fused_observation.input(0) = base_command_.linear.x;
  fused_observation.input(1) = base_command_.angular.z;
  fused_observation.input(2) = arm_command_.linear.x;
  fused_observation.input(3) = arm_command_.linear.y;
  fused_observation.input(4) = arm_command_.linear.z;
  fused_observation.input(5) = arm_command_.angular.x;
  fused_observation.input(6) = arm_command_.angular.y;
  fused_observation.input(7) = arm_command_.angular.z;
  // std::cout << "the entities of the state" << fused_observation.state(3) << std::endl;
}

void ActualMRT::publish_everything()
{
  SystemObservation currentObservation;
  pub_base_command_.publish(base_command_);
  pub_arm_command_.publish(arm_command_);
  fusion(currentObservation);
  mpcObservationMsg_ = ros_msg_conversions::createObservationMsg(currentObservation);
  pub_observation_.publish(mpcObservationMsg_);
}

void ActualMRT::run()
{
  while(n_.ok())
  {
    publish_everything();
  }
}

void ActualMRT::reset(const TargetTrajectories& initTargetTrajectories)
{
  ocs2_msgs::reset resetSrv;
  resetSrv.request.reset = static_cast<uint8_t>(true);
  resetSrv.request.targetTrajectories = ros_msg_conversions::createTargetTrajectoriesMsg(initTargetTrajectories);

  while (!mpcResetServiceClient_.waitForExistence(ros::Duration(5.0)) && ::ros::ok() && ::ros::master::check()) {
    ROS_ERROR_STREAM("Failed to call service to reset MPC, retrying...");
  }

  mpcResetServiceClient_.call(resetSrv);
  ROS_INFO_STREAM("MPC node has been reset.");
}




int main(int argc, char** argv) {
  const std::string robotName = "mobile_manipulator";

  // Initialize ros node
  ros::init(argc, argv, robotName + "_actual_mrt");
  ros::NodeHandle n;

  ActualMRT ActualMRT(n);
  ros::AsyncSpinner spinner(7);
  spinner.start();


  //initialization of observation
  SystemObservation initObservation;
  ActualMRT.fusion(initObservation);
  initObservation.input.setZero(8);
  initObservation.time = 0.0;


  // initial command
  // command的初始化。可以看出初始的init target trajectory只包含一组数据。
  vector_t initTarget(7);
  initTarget.head(3) << 1, 0, 1;
  initTarget.tail(4) << Eigen::Quaternion<scalar_t>(1, 0, 0, 0).coeffs();
  const vector_t zeroInput = vector_t::Zero(8);
  const TargetTrajectories initTargetTrajectories({initObservation.time}, {initTarget}, {zeroInput});

  // Run dummy (loops while ros is ok)
  ActualMRT.reset(initTargetTrajectories);

  ActualMRT.run();

  // Successful exit
  return 0;
}
