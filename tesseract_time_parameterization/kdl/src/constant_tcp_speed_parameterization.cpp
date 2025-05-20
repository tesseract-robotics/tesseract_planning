/**
 * @file contant_tcp_speed_parameterization.cpp
 * @brief Constant Tool Center Point (TCP) speed time parameterization
 *
 * @copyright Copyright (c) 2025, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <tesseract_time_parameterization/kdl/constant_tcp_speed_parameterization.h>
#include <tesseract_time_parameterization/kdl/constant_tcp_speed_parameterization_profiles.h>
#include <tesseract_time_parameterization/core/instructions_trajectory.h>

// Tesseract SDK
#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/profile_dictionary.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_kinematics/core/utils.h>
#include <tesseract_kinematics/kdl/kdl_utils.h>
#include <tesseract_environment/environment.h>
#include <tesseract_command_language/composite_instruction.h>

// KDL SDK
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/path_line.hpp>
#include <kdl/path_composite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/velocityprofile_traphalf.hpp>
#include <kdl/utilities/error.h>

// Eigen SDK
#include <Eigen/Geometry>

Eigen::VectorXd fromKDL(const KDL::Twist& k)
{
  Eigen::VectorXd out(6);
  for (int i = 0; i < 6; ++i)
    out[i] = k[i];
  return out;
}

namespace tesseract_planning
{
Eigen::VectorXd computeJointVelocity(const Eigen::MatrixXd& jac, const KDL::Twist& x_dot, const Eigen::VectorXd& q)
{
  const Eigen::Index dof = q.size();
  Eigen::VectorXd q_dot(dof);
  if (dof == 6)
  {
    q_dot = jac.colPivHouseholderQr().solve(fromKDL(x_dot));
  }
  else
  {
    if (!tesseract_kinematics::solvePInv(jac, fromKDL(x_dot), q_dot))
      throw std::runtime_error("Failed to solve pseudo-inverse for joint velocity calculation");
  }

  return q_dot;
}

Eigen::VectorXd computeJointAcceleration(KDL::ChainJntToJacDotSolver& solver,
                                         const Eigen::MatrixXd& jac,
                                         const KDL::Twist& x_dot_dot,
                                         const Eigen::VectorXd& q,
                                         const Eigen::VectorXd& q_dot)
{
  // Compute the derivative of the Jacobian
  KDL::JntArrayVel jnt_array;
  jnt_array.q.data = q;
  jnt_array.qdot.data = q_dot;
  KDL::Twist jac_dot_q_dot;
  {
    int error = solver.JntToJacDot(jnt_array, jac_dot_q_dot);
    if (error != 0)
      throw std::runtime_error(solver.strError(error));
  }

  // Compute the joint accelerations
  // d/dt(x_dot) = d/dt(J * q_dot)
  // x_dot_dot = J_dot * q_dot + J * q_dot_dot
  // qdotdot = J^(-1) * (x_dot_dot - J_dot_q_dot)
  Eigen::VectorXd twist = fromKDL(x_dot_dot - jac_dot_q_dot);

  Eigen::Index dof = q.size();
  Eigen::VectorXd q_dot_dot(dof);
  if (dof == 6)
  {
    q_dot_dot = jac.colPivHouseholderQr().solve(twist);
  }
  else
  {
    if (!tesseract_kinematics::solvePInv(jac, twist, q_dot_dot))
      throw std::runtime_error("Failed to solve pseudo-inverse for acceleration calculation");
  }

  return q_dot_dot;
}

ConstantTCPSpeedParameterization::ConstantTCPSpeedParameterization(std::string name)
  : TimeParameterization(std::move(name))
{
}

bool ConstantTCPSpeedParameterization::compute(CompositeInstruction& composite_instruction,
                                               const tesseract_environment::Environment& env,
                                               const tesseract_common::ProfileDictionary& profiles) const
{
  auto flattened = composite_instruction.flatten(moveFilter);
  if (flattened.empty())
    return true;

  const tesseract_common::ManipulatorInfo manip_info = composite_instruction.getManipulatorInfo();
  Eigen::Isometry3d tcp_offset = env.findTCPOffset(manip_info);
  auto jg = env.getJointGroup(manip_info.manipulator);

  auto ci_profile = profiles.getProfile<ConstantTCPSpeedParameterizationCompositeProfile>(
      name_,
      composite_instruction.getProfile(name_),
      std::make_shared<ConstantTCPSpeedParameterizationCompositeProfile>());

  // Check profile
  if (!ci_profile)
    throw std::runtime_error("ConstantTCPSpeedParameterization: Invalid composite profile");

  if (ci_profile->max_translational_velocity < 0.0)
    throw std::runtime_error("ConstantTCPSpeedParameterization, max_translational_velocity must be greater than zero!");

  if (ci_profile->max_rotational_velocity < 0.0)
    throw std::runtime_error("ConstantTCPSpeedParameterization, max_rotational_velocity must be greater than zero!");

  if (ci_profile->max_translational_acceleration < 0.0)
    throw std::runtime_error("ConstantTCPSpeedParameterization, max_translational_acceleration must be greater than "
                             "zero!");

  if (ci_profile->max_rotational_acceleration < 0.0)
    throw std::runtime_error("ConstantTCPSpeedParameterization, max_rotational_acceleration must be greater than "
                             "zero!");

  // NOLINTNEXTLINE(readability-simplify-boolean-expr)
  if (!((ci_profile->max_velocity_scaling_factor > 0.0) && (ci_profile->max_velocity_scaling_factor <= 1.0)))
    throw std::runtime_error("ConstantTCPSpeedParameterization, velocity scale factor must be greater than zero!");

  // NOLINTNEXTLINE(readability-simplify-boolean-expr)
  if (!((ci_profile->max_acceleration_scaling_factor > 0.0) && (ci_profile->max_acceleration_scaling_factor <= 1.0)))
    throw std::runtime_error("ConstantTCPSpeedParameterization, acceleration scale factor must be greater than zero!");

  const double eq_radius =
      std::max((ci_profile->max_translational_velocity / ci_profile->max_rotational_velocity),
               (ci_profile->max_translational_acceleration / ci_profile->max_rotational_acceleration));

  // Construct the KDL chain
  tesseract_kinematics::KDLChainData data;
  if (!tesseract_kinematics::parseSceneGraph(data, *env.getSceneGraph(), jg->getBaseLinkName(), manip_info.tcp_frame))
    throw std::runtime_error("Failed to construct KDL chain");

  // Create a Jacobian derivative solver
  KDL::ChainJntToJacDotSolver solver(data.robot_chain);

  try
  {
    InstructionsTrajectory trajectory(composite_instruction);

    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
    auto* path = new KDL::Path_Composite();

    // Create a container for the times of each waypoint in the newly parameterized trajectory
    std::vector<double> times;
    times.reserve(static_cast<std::size_t>(trajectory.size()));
    times.push_back(0.0);

    const double max_vel = ci_profile->max_velocity_scaling_factor * ci_profile->max_translational_velocity;
    const double max_acc = ci_profile->max_acceleration_scaling_factor * ci_profile->max_translational_acceleration;
    KDL::VelocityProfile_TrapHalf v_trap_half(max_vel, max_acc, true);
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
    auto* v_trap = new KDL::VelocityProfile_Trap(max_vel, max_acc);

    for (Eigen::Index i = 1; i < trajectory.size(); ++i)
    {
      const Eigen::VectorXd& start_joints = trajectory.getPosition(i - 1);
      const Eigen::VectorXd& end_joints = trajectory.getPosition(i);

      // Perform FK to get Cartesian poses
      KDL::Frame start;
      tesseract_kinematics::EigenToKDL(jg->calcFwdKin(start_joints).at(manip_info.tcp_frame) * tcp_offset, start);

      KDL::Frame end;
      tesseract_kinematics::EigenToKDL(jg->calcFwdKin(end_joints).at(manip_info.tcp_frame) * tcp_offset, end);

      // Convert to KDL::Path
      // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
      KDL::RotationalInterpolation* rot_interp = new KDL::RotationalInterpolation_SingleAxis();
      // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
      KDL::Path* segment = new KDL::Path_Line(start, end, rot_interp, eq_radius);

      // Add the segment to the full path
      path->Add(segment);

      const double path_length = std::max(path->PathLength(), std::numeric_limits<double>::epsilon());

      /* KDL does not provide the ability to determine the time at which a particular waypoint occurs in velocity
       * profile parameterized trajectory. Therefore, we need to estimate the time to each waypoint. We can do this by
       * applying a half-trapezoid (front) velocity profile to the path constructed so far and calculating the
       * duration.
       */
      v_trap_half.SetProfile(0.0, path_length);
      times.push_back(v_trap_half.Duration());
    }

    // Apply a double-ended trapezoidal velocity profile to the full path
    const double path_length = path->PathLength();
    v_trap->SetProfile(0.0, path_length);
    const double duration = v_trap->Duration();

    // Add the last time with the duration from a double ended trapezoidal velocity profile
    times.back() = duration;

    /* In some cases, the deceleration may not occur completely in the last path segment, so estimating the time to a
     * waypoint using the forward half-trapezoidal velocity profile can be incorrect. Instead we need to work
     * backwards from the end of the trajectory to each waypoint until we reach the halfway point or until we reach a
     * waypoint that is at full speed
     */
    {
      // Construct a reverse path
      KDL::Path_Composite reverse_path;
      const int n_segments = path->GetNrOfSegments();
      const int middle_segment = n_segments / 2;

      // Iterate in reverse through the path until we hit a point whose velocity is
      for (int i = path->GetNrOfSegments(); i-- > middle_segment;)
      {
        reverse_path.Add(path->GetSegment(i), false);

        const double reverse_path_length = std::max(reverse_path.PathLength(), std::numeric_limits<double>::epsilon());
        v_trap_half.SetProfile(0.0, reverse_path_length);
        const double reverse_path_duration = v_trap_half.Duration();
        const double vel = v_trap_half.Vel(reverse_path_duration);

        if (std::abs(vel - max_vel) > std::numeric_limits<double>::epsilon())
        {
          times[static_cast<std::size_t>(i)] = duration - reverse_path_duration;
        }
        else
        {
          break;
        }
      }
    }

    // Update the trajectory
    for (Eigen::Index i = 0; i < trajectory.size(); ++i)
    {
      const Eigen::VectorXd& joints = trajectory.getPosition(i);
      const double t = times[static_cast<std::size_t>(i)];

      // Compute the joint velocity and acceleration
      KDL::Trajectory_Segment traj(path, v_trap, false);
      const KDL::Twist vel = traj.Vel(t);
      const KDL::Twist acc = traj.Acc(t);
      const Eigen::MatrixXd jac = jg->calcJacobian(joints, manip_info.tcp_frame, tcp_offset.translation());
      const Eigen::VectorXd joint_vel = computeJointVelocity(jac, vel, joints);
      const Eigen::VectorXd joint_acc = computeJointAcceleration(solver, jac, acc, joints, joint_vel);

      // Update the trajectory container
      trajectory.setData(i, joint_vel, joint_acc, t);
    }
  }
  catch (KDL::Error& e)
  {
    std::stringstream ss;
    ss << "KDL Error #" << e.GetType() << ": " << e.Description();
    CONSOLE_BRIDGE_logError(ss.str().c_str());
    return false;
  }
  catch (const std::exception& ex)
  {
    CONSOLE_BRIDGE_logError(ex.what());
    return false;
  }

  return true;
}

}  // namespace tesseract_planning
