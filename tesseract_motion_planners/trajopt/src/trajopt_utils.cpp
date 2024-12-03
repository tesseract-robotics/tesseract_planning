/**
 * @file trajopt_utils.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date June 18, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#include <tesseract_motion_planners/trajopt/trajopt_utils.h>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_environment/environment.h>
#include <trajopt/problem_description.hpp>
#include <console_bridge/console.h>

namespace tesseract_planning
{
std::shared_ptr<const tesseract_kinematics::JointGroup>
createKinematicGroup(const tesseract_common::ManipulatorInfo& manip_info, const tesseract_environment::Environment& env)
{
  // Assign Kinematics object
  try
  {
    tesseract_kinematics::KinematicGroup::ConstPtr kin_group;
    std::string error_msg;
    if (manip_info.manipulator_ik_solver.empty())
    {
      kin_group = env.getKinematicGroup(manip_info.manipulator);
      error_msg = "Failed to find kinematic group for manipulator '" + manip_info.manipulator + "'";
    }
    else
    {
      kin_group = env.getKinematicGroup(manip_info.manipulator, manip_info.manipulator_ik_solver);
      error_msg = "Failed to find kinematic group for manipulator '" + manip_info.manipulator + "' with solver '" +
                  manip_info.manipulator_ik_solver + "'";
    }

    if (kin_group == nullptr)
    {
      CONSOLE_BRIDGE_logError("%s", error_msg.c_str());
      throw std::runtime_error(error_msg);
    }

    return kin_group;
  }
  catch (...)
  {
    return env.getJointGroup(manip_info.manipulator);
  }
}

std::shared_ptr<trajopt::TermInfo> createCartesianWaypointTermInfo(int index,
                                                                   const std::string& working_frame,
                                                                   const Eigen::Isometry3d& c_wp,
                                                                   const std::string& tcp_frame,
                                                                   const Eigen::Isometry3d& tcp_offset,
                                                                   const Eigen::VectorXd& coeffs,
                                                                   trajopt::TermType type,
                                                                   const Eigen::VectorXd& lower_tolerance,
                                                                   const Eigen::VectorXd& upper_tolerance)
{
  auto pose_info = std::make_shared<trajopt::CartPoseTermInfo>();
  pose_info->term_type = type;
  pose_info->name = "cartesian_waypoint_" + std::to_string(index);
  pose_info->timestep = index;

  pose_info->source_frame = tcp_frame;
  pose_info->source_frame_offset = tcp_offset;

  pose_info->target_frame = working_frame;
  pose_info->target_frame_offset = c_wp;

  if (coeffs.size() == 1)
  {
    pose_info->pos_coeffs = Eigen::Vector3d::Constant(coeffs(0));
    pose_info->rot_coeffs = Eigen::Vector3d::Constant(coeffs(0));
  }
  else if (coeffs.size() == 6)
  {
    pose_info->pos_coeffs = coeffs.head<3>();
    pose_info->rot_coeffs = coeffs.tail<3>();
  }

  if (lower_tolerance.size() == 1)
  {
    pose_info->lower_tolerance = Eigen::VectorXd::Constant(6, lower_tolerance(0));
  }
  else if (lower_tolerance.size() == 6)
  {
    pose_info->lower_tolerance = lower_tolerance;
  }

  if (upper_tolerance.size() == 1)
  {
    pose_info->upper_tolerance = Eigen::VectorXd::Constant(6, upper_tolerance(0));
  }
  else if (upper_tolerance.size() == 6)
  {
    pose_info->upper_tolerance = upper_tolerance;
  }

  return pose_info;
}

std::shared_ptr<trajopt::TermInfo> createDynamicCartesianWaypointTermInfo(int index,
                                                                          const std::string& working_frame,
                                                                          const Eigen::Isometry3d& c_wp,
                                                                          const std::string& tcp_frame,
                                                                          const Eigen::Isometry3d& tcp_offset,
                                                                          const Eigen::VectorXd& coeffs,
                                                                          trajopt::TermType type,
                                                                          const Eigen::VectorXd& lower_tolerance,
                                                                          const Eigen::VectorXd& upper_tolerance)
{
  std::shared_ptr<trajopt::DynamicCartPoseTermInfo> pose = std::make_shared<trajopt::DynamicCartPoseTermInfo>();
  pose->term_type = type;
  pose->name = "dyn_cartesian_waypoint_" + std::to_string(index);
  pose->timestep = index;

  pose->source_frame = tcp_frame;
  pose->source_frame_offset = tcp_offset;

  pose->target_frame = working_frame;
  pose->target_frame_offset = c_wp;

  if (coeffs.size() == 1)
  {
    pose->pos_coeffs = Eigen::Vector3d::Constant(coeffs(0));
    pose->rot_coeffs = Eigen::Vector3d::Constant(coeffs(0));
  }
  else if (coeffs.size() == 6)
  {
    pose->pos_coeffs = coeffs.head<3>();
    pose->rot_coeffs = coeffs.tail<3>();
  }

  pose->lower_tolerance = lower_tolerance;
  pose->upper_tolerance = upper_tolerance;

  return pose;
}

std::shared_ptr<trajopt::TermInfo> createNearJointStateTermInfo(const Eigen::VectorXd& target,
                                                                const std::vector<std::string>& joint_names,
                                                                int index,
                                                                const Eigen::VectorXd& coeffs,
                                                                trajopt::TermType type)
{
  assert(static_cast<std::size_t>(target.size()) == joint_names.size());

  std::shared_ptr<trajopt::JointPosTermInfo> jp = std::make_shared<trajopt::JointPosTermInfo>();
  if (static_cast<std::size_t>(coeffs.size()) == 1)
    jp->coeffs = std::vector<double>(joint_names.size(), coeffs(0));  // Default value
  else if (static_cast<std::size_t>(coeffs.size()) == joint_names.size())
    jp->coeffs = std::vector<double>(coeffs.data(), coeffs.data() + coeffs.rows() * coeffs.cols());

  jp->targets = std::vector<double>(target.data(), target.data() + target.size());
  jp->first_step = index;
  jp->last_step = index;
  jp->name = "near_state_" + std::to_string(index);
  jp->term_type = type;

  return jp;
}

std::shared_ptr<trajopt::TermInfo> createJointWaypointTermInfo(const Eigen::VectorXd& j_wp,
                                                               int index,
                                                               const Eigen::VectorXd& coeffs,
                                                               trajopt::TermType type)
{
  auto joint_info = std::make_shared<trajopt::JointPosTermInfo>();
  if (coeffs.size() == 1)
    joint_info->coeffs = std::vector<double>(static_cast<std::size_t>(j_wp.size()), coeffs(0));
  else if (coeffs.size() == j_wp.size())
    joint_info->coeffs = std::vector<double>(coeffs.data(), coeffs.data() + coeffs.rows() * coeffs.cols());

  joint_info->targets = std::vector<double>(j_wp.data(), j_wp.data() + j_wp.rows() * j_wp.cols());
  joint_info->first_step = index;
  joint_info->last_step = index;
  joint_info->name = "joint_waypoint_" + std::to_string(index);
  joint_info->term_type = type;

  return joint_info;
}

std::shared_ptr<trajopt::TermInfo> createTolerancedJointWaypointTermInfo(const Eigen::VectorXd& j_wp,
                                                                         const Eigen::VectorXd& lower_tol,
                                                                         const Eigen::VectorXd& upper_tol,
                                                                         int index,
                                                                         const Eigen::VectorXd& coeffs,
                                                                         trajopt::TermType type)
{
  auto joint_info = std::make_shared<trajopt::JointPosTermInfo>();
  if (coeffs.size() == 1)
    joint_info->coeffs = std::vector<double>(static_cast<std::size_t>(j_wp.size()), coeffs(0));
  else if (coeffs.size() == j_wp.size())
    joint_info->coeffs = std::vector<double>(coeffs.data(), coeffs.data() + coeffs.rows() * coeffs.cols());

  joint_info->targets = std::vector<double>(j_wp.data(), j_wp.data() + j_wp.rows() * j_wp.cols());
  joint_info->lower_tols =
      std::vector<double>(lower_tol.data(), lower_tol.data() + lower_tol.rows() * lower_tol.cols());
  joint_info->upper_tols =
      std::vector<double>(upper_tol.data(), upper_tol.data() + upper_tol.rows() * upper_tol.cols());
  joint_info->first_step = index;
  joint_info->last_step = index;
  joint_info->name = "joint_waypoint_" + std::to_string(index);
  joint_info->term_type = type;

  return joint_info;
}

std::shared_ptr<trajopt::TermInfo> createCollisionTermInfo(int start_index,
                                                           int end_index,
                                                           std::vector<int> fixed_steps,
                                                           trajopt_common::TrajOptCollisionConfig collision_config,
                                                           trajopt::TermType type)
{
  std::shared_ptr<trajopt::CollisionTermInfo> collision = std::make_shared<trajopt::CollisionTermInfo>();
  collision->name = "collision";
  collision->term_type = type;
  collision->first_step = start_index;
  collision->last_step = end_index;
  collision->fixed_steps = std::move(fixed_steps);
  collision->config = std::move(collision_config);
  return collision;
}

std::shared_ptr<trajopt::TermInfo>
createSmoothVelocityTermInfo(int start_index, int end_index, int n_joints, double coeff, trajopt::TermType type)
{
  if ((end_index - start_index) < 1)
    throw std::runtime_error("TrajOpt JointVelTermInfo requires at least two states, failed with start_index " +
                             std::to_string(start_index) + " & end_index " + std::to_string(end_index) + "!");

  std::shared_ptr<trajopt::JointVelTermInfo> jv = std::make_shared<trajopt::JointVelTermInfo>();
  jv->coeffs = std::vector<double>(static_cast<std::size_t>(n_joints), coeff);
  jv->targets = std::vector<double>(static_cast<std::size_t>(n_joints), 0.0);
  jv->first_step = start_index;
  jv->last_step = end_index;
  jv->name = "joint_vel_cost";
  jv->term_type = type;
  return jv;
}

std::shared_ptr<trajopt::TermInfo> createSmoothVelocityTermInfo(int start_index,
                                                                int end_index,
                                                                const Eigen::Ref<const Eigen::VectorXd>& coeff,
                                                                trajopt::TermType type)
{
  if ((end_index - start_index) < 1)
    throw std::runtime_error("TrajOpt JointVelTermInfo requires at least two states, failed with start_index " +
                             std::to_string(start_index) + " & end_index " + std::to_string(end_index) + "!");

  std::shared_ptr<trajopt::JointVelTermInfo> jv = std::make_shared<trajopt::JointVelTermInfo>();
  jv->coeffs = std::vector<double>(coeff.data(), coeff.data() + coeff.size());
  jv->targets = std::vector<double>(static_cast<std::size_t>(coeff.size()), 0.0);
  jv->first_step = start_index;
  jv->last_step = end_index;
  jv->name = "joint_vel_cost";
  jv->term_type = type;
  return jv;
}

std::shared_ptr<trajopt::TermInfo>
createSmoothAccelerationTermInfo(int start_index, int end_index, int n_joints, double coeff, trajopt::TermType type)
{
  if ((end_index - start_index) < 2)
    throw std::runtime_error("TrajOpt JointAccTermInfo requires at least three states, failed with start_index " +
                             std::to_string(start_index) + " & end_index " + std::to_string(end_index) + "!");

  std::shared_ptr<trajopt::JointAccTermInfo> ja = std::make_shared<trajopt::JointAccTermInfo>();
  ja->coeffs = std::vector<double>(static_cast<std::size_t>(n_joints), coeff);
  ja->targets = std::vector<double>(static_cast<std::size_t>(n_joints), 0.0);
  ja->first_step = start_index;
  ja->last_step = end_index;
  ja->name = "joint_accel_cost";
  ja->term_type = type;
  return ja;
}

std::shared_ptr<trajopt::TermInfo> createSmoothAccelerationTermInfo(int start_index,
                                                                    int end_index,
                                                                    const Eigen::Ref<const Eigen::VectorXd>& coeff,
                                                                    trajopt::TermType type)
{
  if ((end_index - start_index) < 2)
    throw std::runtime_error("TrajOpt JointAccTermInfo requires at least three states, failed with start_index " +
                             std::to_string(start_index) + " & end_index " + std::to_string(end_index) + "!");

  std::shared_ptr<trajopt::JointAccTermInfo> ja = std::make_shared<trajopt::JointAccTermInfo>();
  ja->coeffs = std::vector<double>(coeff.data(), coeff.data() + coeff.size());
  ja->targets = std::vector<double>(static_cast<std::size_t>(coeff.size()), 0.0);
  ja->first_step = start_index;
  ja->last_step = end_index;
  ja->name = "joint_accel_cost";
  ja->term_type = type;
  return ja;
}

std::shared_ptr<trajopt::TermInfo>
createSmoothJerkTermInfo(int start_index, int end_index, int n_joints, double coeff, trajopt::TermType type)
{
  if ((end_index - start_index) < 4)
    throw std::runtime_error("TrajOpt JointJerkTermInfo requires at least five states, failed with start_index " +
                             std::to_string(start_index) + " & end_index " + std::to_string(end_index) + "!");

  std::shared_ptr<trajopt::JointJerkTermInfo> jj = std::make_shared<trajopt::JointJerkTermInfo>();
  jj->coeffs = std::vector<double>(static_cast<std::size_t>(n_joints), coeff);
  jj->targets = std::vector<double>(static_cast<std::size_t>(n_joints), 0.0);
  jj->first_step = start_index;
  jj->last_step = end_index;
  jj->name = "joint_jerk_cost";
  jj->term_type = type;
  return jj;
}

std::shared_ptr<trajopt::TermInfo> createSmoothJerkTermInfo(int start_index,
                                                            int end_index,
                                                            const Eigen::Ref<const Eigen::VectorXd>& coeff,
                                                            trajopt::TermType type)
{
  if ((end_index - start_index) < 4)
    throw std::runtime_error("TrajOpt JointJerkTermInfo requires at least five states, failed with start_index " +
                             std::to_string(start_index) + " & end_index " + std::to_string(end_index) + "!");

  std::shared_ptr<trajopt::JointJerkTermInfo> jj = std::make_shared<trajopt::JointJerkTermInfo>();
  jj->coeffs = std::vector<double>(coeff.data(), coeff.data() + coeff.size());
  jj->targets = std::vector<double>(static_cast<std::size_t>(coeff.size()), 0.0);
  jj->first_step = start_index;
  jj->last_step = end_index;
  jj->name = "joint_jerk_cost";
  jj->term_type = type;
  return jj;
}

std::shared_ptr<trajopt::TermInfo> createUserDefinedTermInfo(int start_index,
                                                             int end_index,
                                                             sco::VectorOfVector::func error_function,
                                                             sco::MatrixOfVector::func jacobian_function,
                                                             trajopt::TermType type)
{
  if (error_function == nullptr)
  {
    throw std::runtime_error("TrajOpt Planner Config constraint from error function received nullptr!");
  }

  auto ef = std::make_shared<trajopt::UserDefinedTermInfo>();
  ef->name = "user_defined";
  ef->term_type = type;
  ef->first_step = start_index;
  ef->last_step = end_index;
  ef->error_function = std::move(error_function);
  ef->jacobian_function = std::move(jacobian_function);

  return ef;
}

std::shared_ptr<trajopt::TermInfo> createAvoidSingularityTermInfo(int start_index,
                                                                  int end_index,
                                                                  const std::string& link,
                                                                  double coeff,
                                                                  trajopt::TermType type)
{
  auto as = std::make_shared<trajopt::AvoidSingularityTermInfo>();
  as->term_type = type;
  as->link = link;
  as->first_step = start_index;
  as->last_step = end_index;
  as->coeffs = std::vector<double>(1, coeff);
  as->name = "avoid_singularity";

  return as;
}

}  // namespace tesseract_planning
