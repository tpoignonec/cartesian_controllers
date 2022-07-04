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
/*!\file    cartesian_motion_controller.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

// Project
#include "cartesian_controller_base/Utility.h"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include <cartesian_motion_controller/cartesian_motion_controller.h>

// Other
#include <algorithm>

namespace cartesian_motion_controller
{

CartesianMotionController::CartesianMotionController()
: Base::CartesianControllerBase(), m_realtime_target_frame_ptr(nullptr)
{
}

#if defined CARTESIAN_CONTROLLERS_GALACTIC
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianMotionController::on_init()
{
  const auto ret = Base::on_init();
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
#elif defined CARTESIAN_CONTROLLERS_FOXY
controller_interface::return_type CartesianMotionController::init(const std::string & controller_name)
{
  const auto ret = Base::init(controller_name);
  if (ret != controller_interface::return_type::OK)
  {
    return ret;
  }

  return controller_interface::return_type::OK;
}
#endif

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianMotionController::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
  const auto ret = Base::on_configure(previous_state);
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  m_target_frame_subscr = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      "target_frame", 
        rclcpp::SystemDefaultsQoS(), 
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { m_realtime_target_frame_ptr.writeFromNonRT(msg); });

  
  m_target_twist_subscr = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
      "target_twist", 
        rclcpp::SystemDefaultsQoS(), 
        [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) { m_realtime_target_twist_ptr.writeFromNonRT(msg); });

  // DEBUG Publishers
  m_current_pose_publisher =
    get_node()->create_publisher<geometry_msgs::msg::PoseStamped>("follower_controller/current_frame", 10);
  m_target_pose_publisher =
    get_node()->create_publisher<geometry_msgs::msg::PoseStamped>("follower_controller/target_frame", 10);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianMotionController::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
  Base::on_activate(previous_state);

  // Reset simulation with real joint state
  m_current_frame = Base::m_ik_solver->getEndEffectorPose();
  m_current_twist.setZero();

  // Start where we are
  m_target_frame = m_current_frame;
  m_target_twist.setZero();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianMotionController::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
{
  // Stop drifting by sending zero joint velocities
  Base::computeJointControlCmds(ctrl::Vector6D::Zero(), rclcpp::Duration::from_seconds(0));
  Base::writeJointControlCmds();
  Base::on_deactivate(previous_state);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

#if defined CARTESIAN_CONTROLLERS_GALACTIC
controller_interface::return_type CartesianMotionController::update(const rclcpp::Time& time,
                                                                   const rclcpp::Duration& period)
#elif defined CARTESIAN_CONTROLLERS_FOXY
controller_interface::return_type CartesianMotionController::update()
#endif
{
  // Synchronize the internal model and the real robot
  Base::m_ik_solver->synchronizeJointPositions(Base::m_joint_state_pos_handles);


  // Forward Dynamics turns the search for the according joint motion into a
  // control process. So, we control the internal model until we meet the
  // Cartesian target motion. This internal control needs some simulation time
  // steps.
  for (int i = 0; i < Base::m_iterations; ++i)
  {
    // The internal 'simulation time' is deliberately independent of the outer
    // control cycle.
    auto internal_period = rclcpp::Duration::from_seconds(0.02);

    // Compute the motion error = target - current.
    ctrl::Vector6D error, error_derivative;
    computeMotionError(error, error_derivative);

    // Turn Cartesian error into joint motion
    Base::computeJointControlCmds(error, error_derivative, internal_period);
  }

  // Write final commands to the hardware interface
  Base::writeJointControlCmds();



  return controller_interface::return_type::OK;
}


ctrl::Vector6D
CartesianMotionController::computeMotionError()
{
  ctrl::Vector6D error;
  ctrl::Vector6D error_derivative;
  computeMotionError(error, error_derivative);
  return error;
}

void
CartesianMotionController::computeMotionError(ctrl::Vector6D& error, ctrl::Vector6D& error_derivative)
{
  // Retrieve latest target pose and twist
  updateTarget();
  // Compute motion error wrt robot_base_link
  m_current_frame = Base::m_ik_solver->getEndEffectorPose();

  // Transformation from target -> current corresponds to error = target - current
  KDL::Frame error_kdl;
  error_kdl.M = m_target_frame.M * m_current_frame.M.Inverse();
  error_kdl.p = m_target_frame.p - m_current_frame.p;

  // Use Rodrigues Vector for a compact representation of orientation errors
  // Only for angles within [0,Pi)
  KDL::Vector rot_axis = KDL::Vector::Zero();
  double angle    = error_kdl.M.GetRotAngle(rot_axis);   // rot_axis is normalized
  double distance = error_kdl.p.Normalize();

  // Clamp maximal tolerated error.
  // The remaining error will be handled in the next control cycle.
  // Note that this is also the maximal offset that the
  // cartesian_compliance_controller can use to build up a restoring stiffness
  // wrench.
  const double max_angle = 1.0;
  const double max_distance = 1.0;
  angle    = std::clamp(angle,-max_angle,max_angle);
  distance = std::clamp(distance,-max_distance,max_distance);

  // Scale errors to allowed magnitudes
  rot_axis = rot_axis * angle;
  error_kdl.p = error_kdl.p * distance;

  // Reassign values
  error(0) = error_kdl.p.x();
  error(1) = error_kdl.p.y();
  error(2) = error_kdl.p.z();
  error(3) = rot_axis(0);
  error(4) = rot_axis(1);
  error(5) = rot_axis(2);


  // Compute error derivative (target_twist - current_twist)
  double alpha_filter_twist = 1 - exp(-0.001 * 2*M_PI*50); // cutoff freq ~ 50 Hz
  m_current_twist = alpha_filter_twist*m_current_twist + (1-alpha_filter_twist)* Base::m_ik_solver->getEndEffectorVel();
  error_derivative = m_target_twist - m_current_twist;

  return;
}

bool CartesianMotionController::updateTarget() 
{
  auto target_pose_msg = m_realtime_target_frame_ptr.readFromRT();
  bool all_ok = true;
  if (!target_pose_msg || !(*target_pose_msg)) // Else, the buffer is empty
  {
    auto& clock = *node_->get_clock();
    RCLCPP_WARN_THROTTLE(node_->get_logger(), clock, 3000, "No target pose received!");  
    all_ok = false;
  }
  else
  {
     targetFrameCallback(*target_pose_msg);
  }
  // Read target twist
  auto target_twist_msg = m_realtime_target_twist_ptr.readFromRT();
  if (!target_twist_msg || !(*target_twist_msg)) // Else, the buffer is empty
  {
    auto& clock = *node_->get_clock();
    RCLCPP_WARN_THROTTLE(node_->get_logger(), clock, 3000, "No target twist received!");  
    all_ok = false;
  }
  else
  {
     targetTwistCallback(*target_twist_msg);
  }


  //--------------------------------------------------------
  // DEBUG ONLY!
  geometry_msgs::msg::PoseStamped current_pose;
  auto current_time = get_node()->now();
  current_pose.header.stamp    = current_time;
  current_pose.header.frame_id = m_robot_base_link;
  current_pose.pose.position.x = m_current_frame.p.x();
  current_pose.pose.position.y = m_current_frame.p.y();
  current_pose.pose.position.z = m_current_frame.p.z();
  m_current_frame.M.GetQuaternion(current_pose.pose.orientation.x,
                      current_pose.pose.orientation.y,
                      current_pose.pose.orientation.z,
                      current_pose.pose.orientation.w);
  m_current_pose_publisher->publish(current_pose);

  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.stamp    = current_time;
  target_pose.header.frame_id = m_robot_base_link;
  target_pose.pose.position.x = m_target_frame.p.x();
  target_pose.pose.position.y = m_target_frame.p.y();
  target_pose.pose.position.z = m_target_frame.p.z();
  m_target_frame.M.GetQuaternion(target_pose.pose.orientation.x,
                      target_pose.pose.orientation.y,
                      target_pose.pose.orientation.z,
                      target_pose.pose.orientation.w);
  m_target_pose_publisher->publish(target_pose);
  //--------------------------------------------------------

  return all_ok;
}
void CartesianMotionController::targetFrameCallback(std::shared_ptr<geometry_msgs::msg::PoseStamped> target_msg)
{ 
  if (target_msg->header.frame_id != Base::m_robot_base_link)
  {
    auto& clock = *node_->get_clock();
    RCLCPP_WARN_THROTTLE(node_->get_logger(),
        clock, 3000,
        "Got target pose in wrong reference frame. Expected: %s but got %s",
        Base::m_robot_base_link.c_str(),
        target_msg->header.frame_id.c_str());
    return;
  }

  m_target_frame = KDL::Frame(
      KDL::Rotation::Quaternion(
        target_msg->pose.orientation.x,
        target_msg->pose.orientation.y,
        target_msg->pose.orientation.z,
        target_msg->pose.orientation.w),
      KDL::Vector(
        target_msg->pose.position.x,
        target_msg->pose.position.y,
        target_msg->pose.position.z));

  return;
}

void CartesianMotionController::targetTwistCallback(std::shared_ptr<geometry_msgs::msg::TwistStamped> target_twist_msg)
{ 
  if (target_twist_msg->header.frame_id != Base::m_robot_base_link)
  {
    m_target_twist.setZero(); // Zero velocity
    auto& clock = *node_->get_clock();
    RCLCPP_WARN_THROTTLE(node_->get_logger(),
        clock, 3000,
        "Got target twist in wrong reference frame. Expected: %s but got %s",
        Base::m_robot_base_link.c_str(),
        target_twist_msg->header.frame_id.c_str());
    return;
  }
  ctrl::Vector6D raw_target_twist;
  raw_target_twist[1] = target_twist_msg->twist.linear.y;
  raw_target_twist[2] = target_twist_msg->twist.linear.z;
  raw_target_twist[3] = target_twist_msg->twist.angular.x;
  raw_target_twist[4] = target_twist_msg->twist.angular.y;
  raw_target_twist[5] = target_twist_msg->twist.angular.z;

  double alpha_target_twist = 1 - exp(-0.001 * 2*M_PI*50); // cutoff freq ~ 200 Hz
  m_target_twist = alpha_target_twist * raw_target_twist + (1-alpha_target_twist) * m_target_twist;
  return;
}

} // namespace

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(cartesian_motion_controller::CartesianMotionController, controller_interface::ControllerInterface)
