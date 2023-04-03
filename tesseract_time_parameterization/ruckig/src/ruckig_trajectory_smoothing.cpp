/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, PickNik Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/* Author: Jack Center, Wyatt Rees, Andy Zelenak, Stephanie Eng, Levi Armstrong */

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_time_parameterization/ruckig/ruckig_trajectory_smoothing.h>
#include <tesseract_common/kinematic_limits.h>

#include <memory>

#include <ruckig/input_parameter.hpp>
#include <ruckig/ruckig.hpp>
#include <ruckig/trajectory.hpp>

namespace tesseract_planning
{
RuckigTrajectorySmoothing::RuckigTrajectorySmoothing(double duration_extension_fraction,
                                                     double max_duration_extension_factor)
  : duration_extension_fraction_(duration_extension_fraction)
  , max_duration_extension_factor_(max_duration_extension_factor)
{
}

void RuckigTrajectorySmoothing::setDurationExtensionFraction(double duration_extension_fraction)
{
  duration_extension_fraction_ = duration_extension_fraction;
}

void RuckigTrajectorySmoothing::setMaxDurationExtensionFactor(double max_duration_extension_factor)
{
  max_duration_extension_factor_ = max_duration_extension_factor;
}

bool RuckigTrajectorySmoothing::compute(TrajectoryContainer& trajectory,
                                        const double& max_velocity,
                                        const double& max_acceleration,
                                        const double& max_jerk,
                                        double max_velocity_scaling_factor,
                                        double max_acceleration_scaling_factor,
                                        double max_jerk_scaling_factor) const
{
  // NOLINTNEXTLINE(clang-analyzer-core.UndefinedBinaryOperatorResult)
  return compute(trajectory,
                 std::vector<double>(static_cast<std::size_t>(trajectory.dof()), max_velocity),
                 std::vector<double>(static_cast<std::size_t>(trajectory.dof()), max_acceleration),
                 std::vector<double>(static_cast<std::size_t>(trajectory.dof()), max_jerk),
                 max_velocity_scaling_factor,
                 max_acceleration_scaling_factor,
                 max_jerk_scaling_factor);
}

bool RuckigTrajectorySmoothing::compute(TrajectoryContainer& trajectory,
                                        const std::vector<double>& max_velocity,
                                        const std::vector<double>& max_acceleration,
                                        const std::vector<double>& max_jerk,
                                        double max_velocity_scaling_factor,
                                        double max_acceleration_scaling_factor,
                                        double max_jerk_scaling_factor) const
{
  // NOLINTNEXTLINE(clang-analyzer-core.UndefinedBinaryOperatorResult)
  return compute(trajectory,
                 Eigen::Map<const Eigen::VectorXd>(max_velocity.data(), static_cast<long>(max_velocity.size())),
                 Eigen::Map<const Eigen::VectorXd>(max_acceleration.data(), static_cast<long>(max_acceleration.size())),
                 Eigen::Map<const Eigen::VectorXd>(max_jerk.data(), static_cast<long>(max_jerk.size())),
                 max_velocity_scaling_factor,
                 max_acceleration_scaling_factor,
                 max_jerk_scaling_factor);
}

bool RuckigTrajectorySmoothing::compute(TrajectoryContainer& trajectory,
                                        const Eigen::Ref<const Eigen::VectorXd>& max_velocity,
                                        const Eigen::Ref<const Eigen::VectorXd>& max_acceleration,
                                        const Eigen::Ref<const Eigen::VectorXd>& max_jerk,
                                        double max_velocity_scaling_factor,
                                        double max_acceleration_scaling_factor,
                                        double max_jerk_scaling_factor) const
{
  Eigen::VectorXd max_velocity_scaling_factors =
      Eigen::VectorXd::Ones(static_cast<Eigen::Index>(trajectory.size())) * max_velocity_scaling_factor;
  Eigen::VectorXd max_acceleration_scaling_factors =
      Eigen::VectorXd::Ones(static_cast<Eigen::Index>(trajectory.size())) * max_acceleration_scaling_factor;
  Eigen::VectorXd max_jerk_scaling_factors =
      Eigen::VectorXd::Ones(static_cast<Eigen::Index>(trajectory.size())) * max_jerk_scaling_factor;
  // NOLINTNEXTLINE(clang-analyzer-core.UndefinedBinaryOperatorResult)
  return compute(trajectory,
                 max_velocity,
                 max_acceleration,
                 max_jerk,
                 max_velocity_scaling_factors,
                 max_acceleration_scaling_factors,
                 max_jerk_scaling_factors);
}

#ifdef WITH_ONLINE_CLIENT
bool RuckigTrajectorySmoothing::compute(TrajectoryContainer& trajectory,
                                        const Eigen::Ref<const Eigen::VectorXd>& max_velocity,
                                        const Eigen::Ref<const Eigen::VectorXd>& max_acceleration,
                                        const Eigen::Ref<const Eigen::VectorXd>& max_jerk,
                                        const Eigen::Ref<const Eigen::VectorXd>& max_velocity_scaling_factors,
                                        const Eigen::Ref<const Eigen::VectorXd>& max_acceleration_scaling_factors,
                                        const Eigen::Ref<const Eigen::VectorXd>& max_jerk_scaling_factors) const
{
  if (trajectory.size() < 2)
    return true;

  if (max_velocity.size() != trajectory.dof() || max_acceleration.size() != trajectory.dof())
    return false;

  // Create input parameters
  const std::size_t end_index = trajectory.size() - 1;
  ruckig::InputParameter<trajectory.dof()> input;

  input.max_velocity = std::vector<double>(max_velocity.data(), max_velocity.data() + max_velocity.rows());
  input.max_acceleration =
      std::vector<double>(max_acceleration.data(), max_acceleration.data() + max_acceleration.rows());
  //  input.max_jerk = {4.0, 3.0, 2.0};

  {  // Set start position
    const Eigen::VectorXd& position = trajectory.getPosition(static_cast<Eigen::Index>(0));
    const Eigen::VectorXd& velocity = trajectory.getVelocity(static_cast<Eigen::Index>(0));
    const Eigen::VectorXd& accleration = trajectory.getAcceleration(static_cast<Eigen::Index>(0));

    input.current_position = std::vector<double>(position.data(), position.data() + position.rows());

    if (velocity.rows() != 0)
      input.current_velocity = std::vector<double>(velocity.data(), velocity.data() + velocity.rows());

    if (accleration.rows() != 0)
      input.current_acceleration = std::vector<double>(accleration.data(), accleration.data() + accleration.rows());
  }

  {  // Set end position
    const Eigen::VectorXd& position = trajectory.getPosition(static_cast<Eigen::Index>(end_index));
    const Eigen::VectorXd& velocity = trajectory.getVelocity(static_cast<Eigen::Index>(end_index));
    const Eigen::VectorXd& accleration = trajectory.getAcceleration(static_cast<Eigen::Index>(end_index));

    input.target_position = std::vector<double>(position.data(), position.data() + position.rows());

    if (velocity.rows() != 0)
      input.target_velocity = std::vector<double>(velocity.data(), velocity.data() + velocity.rows());

    if (accleration.rows() != 0)
      input.target_acceleration = std::vector<double>(accleration.data(), accleration.data() + accleration.rows());
  }

  // Add intermediate positions
  for (std::size_t j = 1; j < trajectory.size() - 1; j++)
    input.intermediate_positions.push_back(std::vector<double>(position.data(), position.data() + position.rows()));

  // We don't need to pass the control rate (cycle time) when using only offline features
  ruckig::Ruckig<trajectory.dof()> otg;
  ruckig::Trajectory<trajectory.dof()> trajectory;

  // Calculate the trajectory in an offline manner (outside of the control loop)
  ruckig::Result result = otg.calculate(input, trajectory);
  if (result != ruckig::Result::Finished)
    return false;
}
#else

void getNextRuckigInput(ruckig::InputParameter<ruckig::DynamicDOFs>& ruckig_input,
                        TrajectoryContainer& trajectory,
                        Eigen::Index current_index,
                        Eigen::Index next_index,
                        const Eigen::Ref<const Eigen::VectorXd>& max_velocity,
                        const Eigen::Ref<const Eigen::VectorXd>& max_acceleration)
{
  // Set current state
  const auto& current_position = trajectory.getPosition(current_index);
  Eigen::VectorXd& current_velocity = trajectory.getVelocity(current_index);
  Eigen::VectorXd& current_accleration = trajectory.getAcceleration(current_index);

  // clamp due to small numerical errors
  current_velocity = current_velocity.array().min(max_velocity.array()).max((-1.0 * max_velocity).array());
  current_accleration =
      current_accleration.array().min(max_acceleration.array()).max((-1.0 * max_acceleration).array());

  const Eigen::VectorXd& next_position = trajectory.getPosition(next_index);
  Eigen::VectorXd& next_velocity = trajectory.getVelocity(next_index);
  Eigen::VectorXd& next_accleration = trajectory.getAcceleration(next_index);

  // clamp due to small numerical errors
  next_velocity = next_velocity.array().min(max_velocity.array()).max((-1.0 * max_velocity).array());
  next_accleration = next_accleration.array().min(max_acceleration.array()).max((-1.0 * max_acceleration).array());

  // Update input
  ruckig_input.current_position =
      std::vector<double>(current_position.data(), current_position.data() + current_position.rows());
  ruckig_input.current_velocity =
      std::vector<double>(current_velocity.data(), current_velocity.data() + current_velocity.rows());
  ruckig_input.current_acceleration =
      std::vector<double>(current_accleration.data(), current_accleration.data() + current_accleration.rows());

  ruckig_input.target_position = std::vector<double>(next_position.data(), next_position.data() + next_position.rows());
  ruckig_input.target_velocity = std::vector<double>(next_velocity.data(), next_velocity.data() + next_velocity.rows());
  ruckig_input.target_acceleration =
      std::vector<double>(next_accleration.data(), next_accleration.data() + next_accleration.rows());
}

void initializeRuckigState(ruckig::InputParameter<ruckig::DynamicDOFs>& ruckig_input,
                           ruckig::OutputParameter<ruckig::DynamicDOFs>& ruckig_output,
                           TrajectoryContainer& trajectory,
                           const Eigen::Ref<const Eigen::VectorXd>& max_velocity,
                           const Eigen::Ref<const Eigen::VectorXd>& max_acceleration)
{
  // Set current state
  const Eigen::VectorXd& current_position = trajectory.getPosition(0);
  Eigen::VectorXd& current_velocity = trajectory.getVelocity(0);
  Eigen::VectorXd& current_accleration = trajectory.getAcceleration(0);

  // clamp due to small numerical errors
  current_velocity = current_velocity.array().min(max_velocity.array()).max((-1.0 * max_velocity).array());
  current_accleration =
      current_accleration.array().min(max_acceleration.array()).max((-1.0 * max_acceleration).array());

  // Intialize Ruckig state
  ruckig_input.current_position =
      std::vector<double>(current_position.data(), current_position.data() + current_position.rows());
  ruckig_input.current_velocity =
      std::vector<double>(current_velocity.data(), current_velocity.data() + current_velocity.rows());
  ruckig_input.current_acceleration =
      std::vector<double>(current_accleration.data(), current_accleration.data() + current_accleration.rows());

  ruckig_output.new_position = ruckig_input.current_position;
  ruckig_output.new_velocity = ruckig_input.current_velocity;
  ruckig_output.new_acceleration = ruckig_input.current_acceleration;
}

bool RuckigTrajectorySmoothing::compute(TrajectoryContainer& trajectory,
                                        const Eigen::Ref<const Eigen::VectorXd>& max_velocity,
                                        const Eigen::Ref<const Eigen::VectorXd>& max_acceleration,
                                        const Eigen::Ref<const Eigen::VectorXd>& max_jerk,
                                        const Eigen::Ref<const Eigen::VectorXd>& /*max_velocity_scaling_factors*/,
                                        const Eigen::Ref<const Eigen::VectorXd>& /*max_acceleration_scaling_factors*/,
                                        const Eigen::Ref<const Eigen::VectorXd>& /*max_jerk_scaling_factors*/) const
{
  if (trajectory.size() < 2)
    return true;

  if (max_velocity.size() != trajectory.dof() || max_acceleration.size() != trajectory.dof())
    return false;

  // Create input parameters
  const auto dof = static_cast<std::size_t>(trajectory.dof());
  const auto num_waypoints = static_cast<std::size_t>(trajectory.size());
  ruckig::InputParameter<ruckig::DynamicDOFs> ruckig_input{ dof };
  ruckig::OutputParameter<ruckig::DynamicDOFs> ruckig_output{ dof };

  const Eigen::VectorXd max_scaled_velocity =
      max_velocity;  // (max_velocity.array() * max_velocity_scaling_factors.array()).transpose();
  ruckig_input.max_velocity =
      std::vector<double>(max_scaled_velocity.data(), max_scaled_velocity.data() + max_scaled_velocity.rows());

  const Eigen::VectorXd max_scaled_acceleration =
      max_acceleration;  // max_acceleration.array() * max_acceleration_scaling_factors.array();
  ruckig_input.max_acceleration = std::vector<double>(max_scaled_acceleration.data(),
                                                      max_scaled_acceleration.data() + max_scaled_acceleration.rows());

  if (!(max_jerk.array() < 0).all())
  {
    const Eigen::VectorXd max_scaled_jerk = max_jerk;  // max_jerk.array() * max_jerk_scaling_factors.array();
    ruckig_input.max_jerk =
        std::vector<double>(max_scaled_jerk.data(), max_scaled_jerk.data() + max_scaled_jerk.rows());
  }

  // Get origina data
  std::vector<Eigen::VectorXd> original_velocities;
  std::vector<Eigen::VectorXd> original_accelerations;
  Eigen::VectorXd original_duration_from_previous;

  original_velocities.reserve(num_waypoints);
  original_accelerations.reserve(num_waypoints);
  original_duration_from_previous.resize(static_cast<Eigen::Index>(num_waypoints));

  original_velocities.push_back(trajectory.getVelocity(0));
  original_accelerations.push_back(trajectory.getAcceleration(0));
  original_duration_from_previous[0] = trajectory.getTimeFromStart(0);
  double previous_time = original_duration_from_previous[0];
  for (Eigen::Index i = 1; i < static_cast<Eigen::Index>(num_waypoints); ++i)
  {
    original_velocities.push_back(trajectory.getVelocity(i));
    original_accelerations.push_back(trajectory.getAcceleration(i));
    const double current_time = trajectory.getTimeFromStart(i);
    original_duration_from_previous(i) = current_time - previous_time;
    previous_time = current_time;
  }

  // Initialize Ruckig
  double timestep = original_duration_from_previous.sum() / static_cast<double>(num_waypoints - 1);
  auto ruckig_ptr = std::make_unique<ruckig::Ruckig<ruckig::DynamicDOFs> >(dof, timestep);
  initializeRuckigState(ruckig_input, ruckig_output, trajectory, max_scaled_velocity, max_scaled_acceleration);

  // Smooth trajectory
  ruckig::Result ruckig_result{};
  double duration_extension_factor{ 1 };
  bool smoothing_complete{ false };
  while ((duration_extension_factor < max_duration_extension_factor_) && !smoothing_complete)
  {
    for (Eigen::Index waypoint_idx = 0; waypoint_idx < num_waypoints - 1; ++waypoint_idx)
    {
      // Get Next Input
      getNextRuckigInput(
          ruckig_input, trajectory, waypoint_idx, waypoint_idx + 1, max_scaled_velocity, max_scaled_acceleration);

      // Run Ruckig
      ruckig_result = ruckig_ptr->update(ruckig_input, ruckig_output);

      if ((waypoint_idx == num_waypoints - 2) && ruckig_result == ruckig::Result::Finished)
      {
        smoothing_complete = true;
        break;
      }

      // Extend the trajectory duration if Ruckig could not reach the waypoint successfully
      if (ruckig_result != ruckig::Result::Finished)
      {
        duration_extension_factor *= duration_extension_fraction_;
        Eigen::VectorXd new_duration_from_previous = original_duration_from_previous;

        double time_from_start = original_duration_from_previous(0);
        for (Eigen::Index time_stretch_idx = 1; time_stretch_idx < num_waypoints; ++time_stretch_idx)
        {
          assert(time_stretch_idx < original_duration_from_previous.rows());
          const double duration_from_previous =
              duration_extension_factor * original_duration_from_previous(time_stretch_idx);
          new_duration_from_previous(time_stretch_idx) = duration_from_previous;
          time_from_start += duration_from_previous;

          // re-calculate waypoint velocity and acceleration
          timestep = new_duration_from_previous.sum() / static_cast<double>(new_duration_from_previous.rows() - 1);
          Eigen::VectorXd new_velocity =
              (1 / duration_extension_factor) * original_velocities[static_cast<std::size_t>(time_stretch_idx)];
          Eigen::VectorXd new_acceleration = (new_velocity - trajectory.getVelocity(time_stretch_idx - 1)) / timestep;
          trajectory.setData(time_stretch_idx, new_velocity, new_acceleration, time_from_start);
        }
        ruckig_ptr = std::make_unique<ruckig::Ruckig<ruckig::DynamicDOFs> >(dof, timestep);
        initializeRuckigState(ruckig_input, ruckig_output, trajectory, max_scaled_velocity, max_scaled_acceleration);

        // Begin the while() loop again
        break;
      }
    }
  }

  if (ruckig_result != ruckig::Result::Finished)
  {
    CONSOLE_BRIDGE_logError("Ruckig trajectory smoothing failed. Ruckig error: %s", ruckig_result);
    return false;
  }

  return true;
}
#endif
}  // namespace tesseract_planning
