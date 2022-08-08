////////////////////////////////////////////////////////////////////////////////
// Copyright 2019 FZI Research Center for Information Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------
/*!\file    IKSolver.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2016/02/14
 *
 */
//-----------------------------------------------------------------------------

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <algorithm>
#include <cartesian_controller_base/IKSolver.h>
#include <functional>
#include <kdl/framevel.hpp>
#include <kdl/jntarrayvel.hpp>
#include <map>
#include <sstream>

namespace cartesian_controller_base{

  IKSolver::IKSolver()
  {
  }

  IKSolver::~IKSolver(){}


  const KDL::Frame& IKSolver::getEndEffectorPose() const
  {
    return m_end_effector_pose;
  }

  const ctrl::Vector6D& IKSolver::getEndEffectorVel() const
  {
    return m_end_effector_vel;
  }

  const KDL::JntArray& IKSolver::getPositions() const
  {
    return m_current_positions;
  }


  bool IKSolver::setStartState(
    const std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> >&
      joint_pos_handles)
  {
    // Copy into internal buffers.
    for (size_t i = 0; i < joint_pos_handles.size(); ++i)
    {
      // Interface type should be checked by the caller.
      // Add additional plausibility check just in case.
      if (joint_pos_handles[i].get().get_interface_name() == hardware_interface::HW_IF_POSITION)
      {
        m_current_positions(i)     = joint_pos_handles[i].get().get_value();
        m_current_velocities(i)    = 0.0;
        m_current_accelerations(i) = 0.0;
        m_last_positions(i)        = m_current_positions(i);
        m_last_velocities(i)       = m_current_velocities(i);
      }
      else
      {
        return false;
      }

      auto & joint_state_msg = realtime_joint_state_publisher_->msg_;
      joint_state_msg.name = {"1", "2", "3", "4", "5", "6", "7"};
      const auto kUninitializedValue = std::numeric_limits<double>::quiet_NaN();
      joint_state_msg.position.resize(m_number_joints, kUninitializedValue);
      joint_state_msg.velocity.resize(m_number_joints, kUninitializedValue);
      joint_state_msg.effort.resize(m_number_joints, kUninitializedValue);
    }
    return true;
  }


  void IKSolver::synchronizeJointPositions(
    const std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> >&
      joint_pos_handles)
  {
    for (size_t i = 0; i < joint_pos_handles.size(); ++i)
    {
      // Interface type should be checked by the caller.
      // Add additional plausibility check just in case.
      if (joint_pos_handles[i].get().get_interface_name() == hardware_interface::HW_IF_POSITION)
      {
        m_current_positions(i) = joint_pos_handles[i].get().get_value();
        m_last_positions(i)    = m_current_positions(i);
      }
    }
    if (realtime_joint_state_publisher_ && realtime_joint_state_publisher_->trylock())
    {
      auto & joint_state_msg = realtime_joint_state_publisher_->msg_;

      joint_state_msg.header.stamp = m_handle->get_clock()->now();

      // update joint state message and dynamic joint state message
      for (size_t i = 0; i < m_number_joints; ++i)
      {
        joint_state_msg.position[i] = m_current_positions(i);
        joint_state_msg.velocity[i] = m_current_velocities(i);
      }
      realtime_joint_state_publisher_->unlockAndPublish();
    }
  }


  bool IKSolver::init(std::shared_ptr<rclcpp::Node> node_handle,
                      const KDL::Chain& chain,
                      const KDL::JntArray& upper_pos_limits,
                      const KDL::JntArray& lower_pos_limits)
  {
    // Initialize
    m_chain = chain;
    m_number_joints              = m_chain.getNrOfJoints();
    m_current_positions.data     = ctrl::VectorND::Zero(m_number_joints);
    m_current_velocities.data    = ctrl::VectorND::Zero(m_number_joints);
    m_current_accelerations.data = ctrl::VectorND::Zero(m_number_joints);
    m_last_positions.data        = ctrl::VectorND::Zero(m_number_joints);
    m_last_velocities.data       = ctrl::VectorND::Zero(m_number_joints);
    m_upper_pos_limits           = upper_pos_limits;
    m_lower_pos_limits           = lower_pos_limits;

    // Forward kinematics
    m_fk_pos_solver.reset(new KDL::ChainFkSolverPos_recursive(m_chain));
    m_fk_vel_solver.reset(new KDL::ChainFkSolverVel_recursive(m_chain));


  // DEBUG
    m_handle = node_handle;
    joint_state_publisher_ = node_handle->create_publisher<sensor_msgs::msg::JointState>(
      "/follower_controller/joint_states", rclcpp::SystemDefaultsQoS());
    realtime_joint_state_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
        joint_state_publisher_);
        

    return true;
  }

  void IKSolver::updateKinematics()
  {
    // Pose w. r. t. base
    m_fk_pos_solver->JntToCart(m_current_positions,m_end_effector_pose);

    // Absolute velocity w. r. t. base
    KDL::FrameVel vel;
    m_fk_vel_solver->JntToCart(KDL::JntArrayVel(m_current_positions,m_current_velocities),vel);
    m_end_effector_vel[0] = vel.deriv().vel.x();
    m_end_effector_vel[1] = vel.deriv().vel.y();
    m_end_effector_vel[2] = vel.deriv().vel.z();
    m_end_effector_vel[3] = vel.deriv().rot.x();
    m_end_effector_vel[4] = vel.deriv().rot.y();
    m_end_effector_vel[5] = vel.deriv().rot.z();
  }

  void IKSolver::applyJointLimits()
  {
    for (int i = 0; i < m_number_joints; ++i)
    {
      if (std::isnan(m_lower_pos_limits(i)) || std::isnan(m_upper_pos_limits(i)))
      {
        // Joint marked as continuous.
        continue;
      }
      m_current_positions(i) = std::clamp(
          m_current_positions(i),m_lower_pos_limits(i),m_upper_pos_limits(i));
    }
  }

} // namespace
