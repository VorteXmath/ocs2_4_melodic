/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include <ocs2_ros_interfaces/command/TargetTrajectoriesInteractiveMarker.h>

using namespace ocs2;

/**
 * Converts the pose of the interactive marker to TargetTrajectories.
 */
// 目前还不是很清楚SystemObservation的结构。主要问题在于这个mode，和 time都是什么。
// state和input还比较好猜，但是也不是很敢确定。
// 这个target trajectory好像只有一个值啊。初始的input也都置零了。
TargetTrajectories goalPoseToTargetTrajectories(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
                                                const SystemObservation& observation) {
  // time trajectory
  const scalar_array_t timeTrajectory{observation.time};
  // state trajectory: 3 + 4 for desired position vector and orientation quaternion
  const vector_t target = (vector_t(7) << position, orientation.coeffs()).finished();
  const vector_array_t stateTrajectory{target};
  // input trajectory
  const vector_array_t inputTrajectory{vector_t::Zero(observation.input.size())};

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

int main(int argc, char* argv[]) {
  const std::string robotName = "mobile_manipulator";
  ::ros::init(argc, argv, robotName + "_target");
  ::ros::NodeHandle nodeHandle;

  // 看来主要是在这里使能的这个方法，
  // 但是我不是特别清楚，最外层的是在那里，即是在哪里开启的这个节点？
  // 更：注意launch文件的最后一行，下回细心点，没事的。
  TargetTrajectoriesInteractiveMarker targetPoseCommand(nodeHandle, robotName, &goalPoseToTargetTrajectories);
  targetPoseCommand.publishInteractiveMarker();

  // Successful exit
  return 0;
}
