/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Tobias Kunz <tobias@gatech.edu>
 * Date: 05/2012
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * Algorithm details and publications:
 * http://www.golems.org/node/1570
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <Eigen/Geometry>
#include <algorithm>
#include <functional>
#include <unordered_set>
#include <cmath>
#include <vector>
#include <tesseract/common/logging.h>
#include <cassert>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/time_parameterization/totg/time_optimal_trajectory_generation.h>
#include <tesseract/time_parameterization/totg/time_optimal_trajectory_generation_profiles.h>
#include <tesseract/time_parameterization/instructions_trajectory.h>
#include <tesseract/command_language/composite_instruction.h>
#include <tesseract/command_language/poly/instruction_poly.h>
#include <tesseract/command_language/poly/move_instruction_poly.h>
#include <tesseract/command_language/poly/state_waypoint_poly.h>
#include <tesseract/common/kinematic_limits.h>
#include <tesseract/common/manipulator_info.h>
#include <tesseract/common/profile_dictionary.h>
#include <tesseract/common/utils.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/environment/environment.h>

constexpr double EPS = 0.000001;

// Waypoint directions whose dot product is at most this turn back, as MoveIt2 tests them
constexpr double REVERSAL_COS = -1.0 + 1e-5;

/** @brief Whether the path through before, corner and after turns back at corner, as compute and Path both test it */
static bool turnsBack(const Eigen::VectorXd& before, const Eigen::VectorXd& corner, const Eigen::VectorXd& after)
{
  return (corner - before).normalized().dot((after - corner).normalized()) <= REVERSAL_COS;
}

// Corner blends of a smaller radius approach the integration's resolution; the path velocity there is about
// sqrt(amax * radius) anyway, so stopping costs little
constexpr double MIN_BLEND_RADIUS = 1e-4;

namespace tesseract::time_parameterization
{
namespace
{
/**
 * @brief Whether the corner at waypoint corner turns back, or blends at a radius the integration cannot resolve, as
 * Path would blend it within max_deviation
 */
bool stopsAtCorner(const Eigen::VectorXd& before,
                   const Eigen::VectorXd& corner,
                   const Eigen::VectorXd& after,
                   double max_deviation)
{
  if (turnsBack(before, corner, after))
    return true;

  const Eigen::VectorXd start = 0.5 * (before + corner);
  const Eigen::VectorXd end = 0.5 * (corner + after);
  const double start_distance = (corner - start).norm();
  const double end_distance = (end - corner).norm();
  if (start_distance < 0.000001 || end_distance < 0.000001)
    return false;

  const Eigen::VectorXd start_direction = (corner - start) / start_distance;
  const Eigen::VectorXd end_direction = (end - corner) / end_distance;
  const double cos_angle = start_direction.dot(end_direction);
  if ((start_direction - end_direction).norm() < 0.000001)
    return false;

  const double angle = std::acos(std::max(-1.0, cos_angle));
  const double deviation_distance = max_deviation * std::sin(0.5 * angle) / (1.0 - std::cos(0.5 * angle));
  const double distance = std::min({ start_distance, end_distance, deviation_distance });
  return distance / std::tan(0.5 * angle) < MIN_BLEND_RADIUS;
}

/** @brief Whether any non-move instruction lies between the moves moves[first] and moves[second] */
bool hasInstructionsBetween(
    const tesseract::command_language::CompositeInstruction& composite,
    const std::vector<std::reference_wrapper<tesseract::command_language::InstructionPoly>>& moves,
    std::size_t first,
    std::size_t second)
{
  const auto instructions = composite.flatten();
  std::size_t move = 0;
  std::size_t first_index{ 0 };
  for (std::size_t i = 0; i < instructions.size() && move <= second; ++i)
  {
    if (&instructions[i].get() != &moves[move].get())
      continue;
    if (move == first)
      first_index = i;
    if (move == second)
      return (i - first_index) != (second - first);
    ++move;
  }
  return false;
}

/** @brief Remove the dropped instructions from the composite, at any depth */
void pruneComposite(tesseract::command_language::CompositeInstruction& composite,
                    const std::unordered_set<const tesseract::command_language::InstructionPoly*>& dropped)
{
  // Each element is tested at its own address before the vector holding it is replaced
  std::vector<tesseract::command_language::InstructionPoly> remaining;
  remaining.reserve(composite.size());
  for (auto& instruction : composite)
  {
    if (instruction.isCompositeInstruction())
      pruneComposite(instruction.as<tesseract::command_language::CompositeInstruction>(), dropped);
    if (dropped.count(&instruction) == 0)
      remaining.push_back(std::move(instruction));
  }
  composite.setInstructions(std::move(remaining));
}

/** @brief Remove from the composite, at any depth, every move in moves that is not listed in kept */
void removeDroppedWaypoints(
    tesseract::command_language::CompositeInstruction& composite,
    const std::vector<std::reference_wrapper<tesseract::command_language::InstructionPoly>>& moves,
    const std::vector<std::size_t>& kept)
{
  if (kept.size() == moves.size())
    return;

  std::unordered_set<const tesseract::command_language::InstructionPoly*> dropped;
  for (const auto& move : moves)
    dropped.insert(&move.get());
  for (std::size_t index : kept)
    dropped.erase(&moves[index].get());

  pruneComposite(composite, dropped);
}
}  // namespace

TimeOptimalTrajectoryGeneration::TimeOptimalTrajectoryGeneration(std::string name)
  : TimeParameterization(std::move(name))
{
}

bool TimeOptimalTrajectoryGeneration::compute(tesseract::command_language::CompositeInstruction& composite_instruction,
                                              const tesseract::environment::Environment& env,
                                              const tesseract::common::ProfileDictionary& profiles) const
{
  auto flattened = composite_instruction.flatten(tesseract::command_language::moveFilter);
  if (flattened.empty())
    return true;

  const tesseract::common::ManipulatorInfo manip_info = composite_instruction.getManipulatorInfo();
  auto jg = env.getJointGroup(manip_info.manipulator);
  tesseract::common::KinematicLimits limits = jg->getLimits();
  Eigen::MatrixX2d velocity_limits{ limits.velocity_limits };
  Eigen::MatrixX2d acceleration_limits{ limits.acceleration_limits };

  auto ci_profile = profiles.getProfile<TimeOptimalTrajectoryGenerationCompositeProfile>(
      name_,
      composite_instruction.getProfile(name_),
      std::make_shared<TimeOptimalTrajectoryGenerationCompositeProfile>());
  if (!ci_profile)
    throw std::runtime_error("TimeOptimalTrajectoryGeneration: Invalid composite profile");

  if (ci_profile->override_limits)
  {
    velocity_limits = ci_profile->velocity_limits;
    acceleration_limits = ci_profile->acceleration_limits;
  }

  // Set scaling factors
  // NOLINTNEXTLINE(readability-simplify-boolean-expr)
  if (!((ci_profile->max_velocity_scaling_factor > 0.0) && (ci_profile->max_velocity_scaling_factor <= 1.0)))
    throw std::runtime_error("TimeOptimalTrajectoryGeneration, velocity scale factor must be in (0, 1]!");

  // NOLINTNEXTLINE(readability-simplify-boolean-expr)
  if (!((ci_profile->max_acceleration_scaling_factor > 0.0) && (ci_profile->max_acceleration_scaling_factor <= 1.0)))
    throw std::runtime_error("TimeOptimalTrajectoryGeneration, acceleration scale factor must be in (0, 1]!");

  // Without a positive tolerance the core leaves corners unblended, which no finite acceleration can follow
  if (!(ci_profile->path_tolerance > 0.0))
    throw std::runtime_error("TimeOptimalTrajectoryGeneration, path tolerance must be greater than zero!");

  // With NaN or infinity every waypoint would repeat the start; below 1e-5 dedup keeps segments too short for the
  // corner blends
  if (!std::isfinite(ci_profile->min_angle_change) || ci_profile->min_angle_change < 1e-5)
    throw std::runtime_error("TimeOptimalTrajectoryGeneration, min angle change must be finite and at least 1e-5!");

  // The core takes one symmetric limit per joint, so each joint gets the tighter of its two sides, scaled
  const Eigen::Index num_joints = jg->numJoints();
  const auto tighter_side = [num_joints](const Eigen::MatrixX2d& limits, const std::string& kind, double scale) {
    const std::string error = "TimeOptimalTrajectoryGeneration, " + kind + " limits must ";
    if (limits.rows() != num_joints)
      throw std::runtime_error(error + "have one row per joint!");

    // Written as a negation so a NaN limit fails too
    if (!((limits.col(0).array() <= 0.0).all() && (limits.col(1).array() >= 0.0).all()))
      throw std::runtime_error(error + "have lower <= 0 <= upper!");

    // The core compares curves derived from the limits, which an infinite limit leaves to rounding and NaN
    if (!limits.allFinite())
      throw std::runtime_error(error + "be finite!");

    return Eigen::VectorXd((-limits.col(0)).cwiseMin(limits.col(1)) * scale);
  };
  const Eigen::VectorXd max_velocity =
      tighter_side(velocity_limits, "velocity", ci_profile->max_velocity_scaling_factor);
  const Eigen::VectorXd max_acceleration =
      tighter_side(acceleration_limits, "acceleration", ci_profile->max_acceleration_scaling_factor);

  // The limits and the loops below index every waypoint by the group's joints
  const std::vector<tesseract::common::JointId>& group_joint_ids = jg->getJointIds();
  const Eigen::VectorXd& first_position = flattened.front()
                                              .get()
                                              .as<tesseract::command_language::MoveInstructionPoly>()
                                              .getWaypoint()
                                              .as<tesseract::command_language::StateWaypointPoly>()
                                              .getPosition();
  Eigen::Array<bool, Eigen::Dynamic, 1> moving = Eigen::Array<bool, Eigen::Dynamic, 1>::Constant(num_joints, false);
  for (std::size_t i = 0; i < flattened.size(); ++i)
  {
    const auto& swp = flattened[i]
                          .get()
                          .as<tesseract::command_language::MoveInstructionPoly>()
                          .getWaypoint()
                          .as<tesseract::command_language::StateWaypointPoly>();
    if (swp.getPosition().size() != num_joints)
    {
      TESSERACT_LOG_ERROR("TimeOptimalTrajectoryGeneration, waypoint {} has {} positions for a group of {} joints.",
                          i,
                          swp.getPosition().size(),
                          num_joints);
      return false;
    }

    if (!swp.getPosition().allFinite())
    {
      TESSERACT_LOG_ERROR("TimeOptimalTrajectoryGeneration, waypoint {} has a position that is not finite.", i);
      return false;
    }

    // Waypoints without joint ids are taken to be in the group's order
    if (!swp.getJointIds().empty() && swp.getJointIds() != group_joint_ids)
    {
      TESSERACT_LOG_ERROR("TimeOptimalTrajectoryGeneration, waypoint {} joint ids differ from group '{}' in name or "
                          "order.",
                          i,
                          manip_info.manipulator);
      return false;
    }

    // Exact, since any motion at all reaches the core's tangent
    moving = moving || (swp.getPosition().array() != first_position.array());
  }

  // A zero limit locks its joint, which is harmless on a joint the program never moves. On one that it moves at all the
  // core divides by the joint's tangent component and stalls or never terminates, so the program cannot be timed
  for (Eigen::Index j = 0; j < num_joints; ++j)
  {
    if (!moving[j] || (max_velocity[j] > 0.0 && max_acceleration[j] > 0.0))
      continue;

    TESSERACT_LOG_ERROR("TimeOptimalTrajectoryGeneration, the program moves joint '{}', whose tighter {} limit is "
                        "zero.",
                        group_joint_ids[static_cast<std::size_t>(j)].name(),
                        !(max_velocity[j] > 0.0) ? "velocity" : "acceleration");
    return false;
  }

  InstructionsTrajectory trajectory(flattened);
  const auto num_points = static_cast<std::size_t>(trajectory.size());
  const std::size_t last = num_points - 1;

  // This lib does not actually work properly when angles wrap around, so we need to unwind the path first
  //  trajectory.unwind(); /// @todo

  // A waypoint within min_angle_change of the last kept point in every joint repeats it
  // (https://github.com/tobiaskunz/trajectories/issues/3), so the path gets one point per run of such waypoints, and
  // the output keeps one waypoint per path point
  const double min_angle_change = ci_profile->min_angle_change;
  const auto repeats = [min_angle_change](const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
    return !((a - b).array().abs() > min_angle_change).any();
  };
  std::vector<Eigen::VectorXd> points;
  std::vector<std::size_t> kept;
  for (std::size_t p = 0; p < num_points; ++p)
  {
    const Eigen::VectorXd& position = trajectory.getPosition(static_cast<Eigen::Index>(p));
    if (p == 0 || !repeats(position, points.back()))
    {
      points.push_back(position);
      kept.push_back(p);
    }
  }

  // The goal is the last waypoint, so a final run keeps its last waypoint instead of its first
  if (kept.back() != last)
  {
    const Eigen::VectorXd& goal = trajectory.getPosition(static_cast<Eigen::Index>(last));
    if (points.size() > 1)
    {
      // The goal replaces the last kept point, and so does any point before that it then repeats, down to the point
      // after the start
      std::size_t first_replaced = points.size() - 1;
      while (first_replaced > 1 && repeats(points[first_replaced - 1], goal))
        --first_replaced;

      // Dropping earlier waypoints for the goal must not move other instructions, such as I/O, ahead of the motion
      if (!hasInstructionsBetween(composite_instruction, flattened, kept[first_replaced], last))
      {
        points.resize(first_replaced);
        kept.resize(first_replaced);
        points.push_back(goal);
        kept.push_back(last);
      }
    }
    else if ((goal.array() != points.front().array()).any())
    {
      // Every waypoint is within min_angle_change of the start, but the motion still ends somewhere else
      points.push_back(goal);
      kept.push_back(last);
    }
  }

  // The profile can ask to start at the first waypoint's velocity, which must point along the path's first direction;
  // the trajectory then starts at its part along that direction
  double start_path_velocity{ 0 };
  const Eigen::VectorXd& start_velocity = trajectory.getVelocity(static_cast<Eigen::Index>(kept.front()));
  if (ci_profile->use_start_velocity && start_velocity.size() != 0)
  {
    if (start_velocity.size() != num_joints || !start_velocity.allFinite())
    {
      TESSERACT_LOG_ERROR("TimeOptimalTrajectoryGeneration, the start velocity must be finite, with one value per "
                          "joint.");
      return false;
    }
  }
  if (ci_profile->use_start_velocity && start_velocity.size() != 0 && !(start_velocity.norm() <= 1e-9))
  {
    if (points.size() == 1)
    {
      TESSERACT_LOG_ERROR("TimeOptimalTrajectoryGeneration, the program does not move, so it cannot start moving.");
      return false;
    }
    const Eigen::VectorXd direction = (points[1] - points[0]).normalized();
    const double along = start_velocity.dot(direction);
    const double sideways = (start_velocity - along * direction).norm();
    if (!(along > 0.0) || sideways > 1e-9 + (1e-3 * along))
    {
      TESSERACT_LOG_ERROR("TimeOptimalTrajectoryGeneration, the start velocity must point forward along the path's "
                          "first segment.");
      return false;
    }
    if ((((along * direction).cwiseAbs() - max_velocity).array() > 1e-12 * max_velocity.array()).any())
    {
      TESSERACT_LOG_ERROR("TimeOptimalTrajectoryGeneration, the start velocity exceeds the velocity limits.");
      return false;
    }
    start_path_velocity = along;
  }

  // Nothing moves; the start stays, as callers that prepend a start waypoint remove the first again
  if (points.size() == 1)
  {
    TESSERACT_LOG_DEBUG("Trajectory is parameterized with 0.0 dynamics since it only contains a single distinct "
                        "waypoint.");
    trajectory.setData(static_cast<Eigen::Index>(kept.front()),
                       Eigen::VectorXd::Zero(num_joints),
                       Eigen::VectorXd::Zero(num_joints),
                       0);
    removeDroppedWaypoints(composite_instruction, flattened, kept);
    return true;
  }

  // A corner that turns back, or is too sharp to blend at a size the integration resolves, is passed at rest: the path
  // is cut there, and each piece is timed from rest to rest
  std::vector<std::size_t> cuts{ 0 };
  for (std::size_t i = 1; i + 1 < points.size(); ++i)
  {
    if (stopsAtCorner(points[i - 1], points[i], points[i + 1], ci_profile->path_tolerance))
      cuts.push_back(i);
  }
  cuts.push_back(points.size() - 1);

  std::vector<std::unique_ptr<totg::Trajectory>> pieces;
  for (std::size_t k = 0; k + 1 < cuts.size(); ++k)
  {
    const auto piece_begin = points.begin() + static_cast<std::ptrdiff_t>(cuts[k]);
    const auto piece_end = points.begin() + static_cast<std::ptrdiff_t>(cuts[k + 1]) + 1;
    const totg::Path path(std::list<Eigen::VectorXd>(piece_begin, piece_end), ci_profile->path_tolerance);
    pieces.push_back(std::make_unique<totg::Trajectory>(
        path, max_velocity, max_acceleration, 0.001, k == 0 ? start_path_velocity : 0.0));
    if (!pieces.back()->isValid())
    {
      TESSERACT_LOG_ERROR("Unable to parameterize trajectory.");
      return false;
    }
  }

  // A piece that fails after others have written restores what they wrote
  struct WrittenState
  {
    Eigen::VectorXd position, velocity, acceleration;
    double time;
  };
  std::vector<WrittenState> before_writing;
  for (std::size_t index : kept)
  {
    const auto i = static_cast<Eigen::Index>(index);
    before_writing.push_back({ trajectory.getPosition(i),
                               trajectory.getVelocity(i),
                               trajectory.getAcceleration(i),
                               trajectory.getTimeFromStart(i) });
  }
  const auto restore = [&]() {
    for (std::size_t j = 0; j < kept.size(); ++j)
    {
      trajectory.getPosition(static_cast<Eigen::Index>(kept[j])) = before_writing[j].position;
      trajectory.setData(static_cast<Eigen::Index>(kept[j]),
                         before_writing[j].velocity,
                         before_writing[j].acceleration,
                         before_writing[j].time);
    }
  };

  // Each piece starts where the one before stops; the waypoint they share is written by the later one, at rest with
  // the later piece's starting acceleration
  double time_offset{ 0 };
  for (std::size_t k = 0; k < pieces.size(); ++k)
  {
    const std::vector<std::size_t> piece_kept(kept.begin() + static_cast<std::ptrdiff_t>(cuts[k]),
                                              kept.begin() + static_cast<std::ptrdiff_t>(cuts[k + 1]) + 1);
    if (!pieces[k]->assignData(trajectory, piece_kept, time_offset))
    {
      restore();
      return false;
    }
    time_offset += pieces[k]->getDuration();
  }

  removeDroppedWaypoints(composite_instruction, flattened, kept);
  return true;
}

namespace totg
{
class LinearPathSegment : public PathSegment
{
public:
  LinearPathSegment(const Eigen::VectorXd& start, const Eigen::VectorXd& end)
    : PathSegment((end - start).norm()), end_(end), start_(start)
  {
  }

  Eigen::VectorXd getConfig(double s) const override
  {
    s /= length_;
    s = std::max(0.0, std::min(1.0, s));
    return (1.0 - s) * start_ + s * end_;
  }

  Eigen::VectorXd getTangent(double /* s */) const override { return (end_ - start_) / length_; }

  Eigen::VectorXd getCurvature(double /* s */) const override { return Eigen::VectorXd::Zero(start_.size()); }

  std::list<double> getSwitchingPoints() const override { return {}; }

  std::unique_ptr<PathSegment> clone() const override { return std::make_unique<LinearPathSegment>(*this); }

private:
  Eigen::VectorXd end_;
  Eigen::VectorXd start_;
};

class CircularPathSegment : public PathSegment
{
public:
  CircularPathSegment(const Eigen::VectorXd& start,
                      const Eigen::VectorXd& intersection,
                      const Eigen::VectorXd& end,
                      double max_deviation)
  {
    if ((intersection - start).norm() < 0.000001 || (end - intersection).norm() < 0.000001)
    {
      length_ = 0.0;
      radius = 1.0;
      center = intersection;
      x = Eigen::VectorXd::Zero(start.size());
      y = Eigen::VectorXd::Zero(start.size());
      return;
    }

    const Eigen::VectorXd start_direction = (intersection - start).normalized();
    const Eigen::VectorXd end_direction = (end - intersection).normalized();

    if ((start_direction - end_direction).norm() < 0.000001)
    {
      length_ = 0.0;
      radius = 1.0;
      center = intersection;
      x = Eigen::VectorXd::Zero(start.size());
      y = Eigen::VectorXd::Zero(start.size());
      return;
    }

    // directions must be different at this point so angle is always non-zero
    // Calls to acos can result in nan values if not careful due to numerical floating point errors.
    // If there is a possibility of calling acos on values that are approximately -1.
    // Then it will result in nan, which leads to a segfault
    // This was solved by addeding std::max(-1.0, start_direction.dot(end_direction))
    // See https://github.com/ros-planning/moveit/pull/1861 for more details
    const double angle = acos(std::max(-1.0, start_direction.dot(end_direction)));
    const double start_distance = (start - intersection).norm();
    const double end_distance = (end - intersection).norm();

    // enforce max deviation
    // The paper multiplies start_distance and end_distance by 0.5 but the original implementation
    // does not.
    double l1 = start_distance;
    double l2 = end_distance;
    double l3 = max_deviation * sin(0.5 * angle) / (1.0 - cos(0.5 * angle));
    double distance = std::min(l1, l2);
    distance = std::min(distance, l3);

    radius = distance / tan(0.5 * angle);
    length_ = angle * radius;

    center = intersection + (end_direction - start_direction).normalized() * radius / cos(0.5 * angle);
    x = (intersection - distance * start_direction - center).normalized();
    y = start_direction;
  }

  Eigen::VectorXd getConfig(double s) const override
  {
    const double angle = s / radius;
    return center + radius * (x * cos(angle) + y * sin(angle));
  }

  Eigen::VectorXd getTangent(double s) const override
  {
    const double angle = s / radius;
    return -x * sin(angle) + y * cos(angle);
  }

  Eigen::VectorXd getCurvature(double s) const override
  {
    const double angle = s / radius;
    return (-1.0 / radius) * (x * cos(angle) + y * sin(angle));
  }

  std::list<double> getSwitchingPoints() const override
  {
    std::list<double> switching_points;
    const Eigen::Index dim = x.size();
    for (Eigen::Index i = 0; i < dim; ++i)
    {
      double switching_angle = atan2(y[i], x[i]);
      if (switching_angle < 0.0)
      {
        switching_angle += M_PI;
      }
      const double switching_point = switching_angle * radius;
      if (switching_point < length_)
      {
        switching_points.push_back(switching_point);
      }
    }
    switching_points.sort();
    return switching_points;
  }

  std::unique_ptr<PathSegment> clone() const override { return std::make_unique<CircularPathSegment>(*this); }

private:
  double radius;
  Eigen::VectorXd center;
  Eigen::VectorXd x;
  Eigen::VectorXd y;
};

Path::Path(const std::list<Eigen::VectorXd>& path, double max_deviation)
{
  if (path.size() < 2)
    throw std::invalid_argument("A TOTG path needs at least two waypoints, got " + std::to_string(path.size()));
  auto path_iterator1 = path.begin();
  auto path_iterator2 = path_iterator1;
  ++path_iterator2;
  std::list<Eigen::VectorXd>::const_iterator path_iterator3;
  Eigen::VectorXd start_config = *path_iterator1;
  mapping_.reserve(path.size());
  mapping_.push_back(0);
  blended_.push_back(false);
  double l{ 0 };
  while (path_iterator2 != path.end())
  {
    path_iterator3 = path_iterator2;
    ++path_iterator3;
    if (path_iterator3 != path.end() && turnsBack(*path_iterator1, *path_iterator2, *path_iterator3))
      has_reversal_ = true;

    if (max_deviation > 0.0 && path_iterator3 != path.end())
    {
      auto blend_segment = std::make_unique<CircularPathSegment>(0.5 * (*path_iterator1 + *path_iterator2),
                                                                 *path_iterator2,
                                                                 0.5 * (*path_iterator2 + *path_iterator3),
                                                                 max_deviation);
      Eigen::VectorXd end_config = blend_segment->getConfig(0.0);
      if ((end_config - start_config).norm() > 0.000001)
      {
        path_segments_.push_back(std::make_unique<LinearPathSegment>(start_config, end_config));
        l += path_segments_.back()->getLength();
      }
      start_config = blend_segment->getConfig(blend_segment->getLength());

      mapping_.push_back(l + (blend_segment->getLength() / 2.0));
      blended_.push_back(blend_segment->getLength() > 0.0);
      l += blend_segment->getLength();
      path_segments_.push_back(std::move(blend_segment));
    }
    else
    {
      path_segments_.push_back(std::make_unique<LinearPathSegment>(start_config, *path_iterator2));
      l += path_segments_.back()->getLength();
      mapping_.push_back(l);
      blended_.push_back(false);
      start_config = *path_iterator2;
    }
    path_iterator1 = path_iterator2;
    ++path_iterator2;
  }
  assert(mapping_.size() == path.size());

  // Create list of switching point candidates, calculate total path length and
  // absolute positions of path segments
  for (std::unique_ptr<PathSegment>& path_segment : path_segments_)
  {
    path_segment->position_ = length_;
    std::list<double> local_switching_points = path_segment->getSwitchingPoints();
    for (const auto& local_switching_point : local_switching_points)
    {
      switching_points_.emplace_back(length_ + local_switching_point, false);
    }
    length_ += path_segment->getLength();
    while (!switching_points_.empty() && switching_points_.back().first >= length_)
      switching_points_.pop_back();
    switching_points_.emplace_back(length_, true);
  }
  switching_points_.pop_back();
  cursor_ = path_segments_.begin();
}

Path::Path(const Path& path)
  : length_(path.length_)
  , mapping_(path.mapping_)
  , blended_(path.blended_)
  , switching_points_(path.switching_points_)
  , has_reversal_(path.has_reversal_)
{
  for (const std::unique_ptr<PathSegment>& path_segment : path.path_segments_)
    path_segments_.emplace_back(path_segment->clone());
  cursor_ = path_segments_.begin();
}

double Path::getLength() const { return length_; }

const std::vector<double>& Path::getMapping() const { return mapping_; }

bool Path::isBlended(std::size_t i) const { return blended_.at(i); }

PathSegment* Path::getPathSegment(double& s) const
{
  // Find the last segment that starts at or before s, resuming from the last one found. The start positions are running
  // sums of lengths, so s >= position holds on a prefix of the segments for any s, NaN included, and this finds the
  // segment a scan from the first one finds
  auto it = cursor_;
  while (it != path_segments_.begin() && !(s >= (*it)->position_))
    --it;
  auto next = it;
  ++next;
  while (next != path_segments_.end() && s >= (*next)->position_)
  {
    it = next;
    ++next;
  }
  cursor_ = it;
  s -= (*it)->position_;
  return (*it).get();
}

Eigen::VectorXd Path::getConfig(double s) const
{
  const PathSegment* path_segment = getPathSegment(s);
  return path_segment->getConfig(s);
}

Eigen::VectorXd Path::getTangent(double s) const
{
  const PathSegment* path_segment = getPathSegment(s);
  return path_segment->getTangent(s);
}

Eigen::VectorXd Path::getCurvature(double s) const
{
  const PathSegment* path_segment = getPathSegment(s);
  return path_segment->getCurvature(s);
}

double Path::getNextSwitchingPoint(double s, bool& discontinuity) const
{
  auto it = switching_points_.begin();
  while (it != switching_points_.end() && it->first <= s)
  {
    ++it;
  }
  if (it == switching_points_.end())
  {
    discontinuity = true;
    return length_;
  }
  discontinuity = it->second;
  return it->first;
}

const std::list<std::pair<double, bool>>& Path::getSwitchingPoints() const& { return switching_points_; }

std::list<std::pair<double, bool>> Path::getSwitchingPoints() const&& { return switching_points_; }

bool Path::hasReversal() const { return has_reversal_; }

Trajectory::Trajectory(const Path& path,
                       const Eigen::VectorXd& max_velocity,
                       const Eigen::VectorXd& max_acceleration,  // NOLINT
                       double time_step)
  : Trajectory(path, max_velocity, max_acceleration, time_step, 0.0)
{
}

Trajectory::Trajectory(const Path& path,
                       const Eigen::VectorXd& max_velocity,
                       const Eigen::VectorXd& max_acceleration,  // NOLINT
                       double time_step,
                       double start_path_velocity)
  : path_(path)
  , max_velocity_(max_velocity)
  , max_acceleration_(max_acceleration)
  , joint_num_(max_velocity.size())
  , time_step_(time_step)
{
  trajectory_.emplace_back(0.0, 0.0);

  // A non-positive step never advances the integration
  if (!(time_step_ > 0.0))
  {
    valid_ = false;
    TESSERACT_LOG_ERROR("Trajectory time step must be greater than zero, got {}", time_step_);
    return;
  }

  // The path velocity must reach zero where the path turns back, which the integration does not model
  if (path_.hasReversal())
  {
    valid_ = false;
    TESSERACT_LOG_ERROR("Trajectory path turns back on itself; split it there and time the pieces from rest to rest");
    return;
  }

  // The first step is built at rest, so its checks pass, and takes the start velocity once that is known to be valid
  if (!(start_path_velocity >= 0.0) || !std::isfinite(start_path_velocity))
  {
    valid_ = false;
    TESSERACT_LOG_ERROR("Trajectory start path velocity must be finite and not negative, got {}", start_path_velocity);
    return;
  }
  if (start_path_velocity > 0.0)
  {
    const double start_limit = std::min(getVelocityMaxPathVelocity(0.0), getAccelerationMaxPathVelocity(0.0));
    if (start_path_velocity > start_limit * (1.0 + 1e-12))
    {
      valid_ = false;
      TESSERACT_LOG_ERROR("Trajectory start path velocity {} exceeds the limit {} at the start of the path",
                          start_path_velocity,
                          start_limit);
      return;
    }
    trajectory_.front().path_vel_ = std::min(start_path_velocity, start_limit);
  }

  double after_acceleration = getMinMaxPathAcceleration(0.0, trajectory_.front().path_vel_, true);
  while (valid_ && !integrateForward(trajectory_, after_acceleration) && valid_)
  {
    double before_acceleration{ NAN };
    TrajectoryStep switching_point;
    if (getNextSwitchingPoint(trajectory_.back().path_pos_, switching_point, before_acceleration, after_acceleration))
    {
      break;
    }
    integrateBackward(trajectory_, switching_point.path_pos_, switching_point.path_vel_, before_acceleration);
  }

  if (valid_)
  {
    double before_acceleration = getMinMaxPathAcceleration(path_.getLength(), 0.0, false);
    integrateBackward(trajectory_, path_.getLength(), 0.0, before_acceleration);
  }

  if (valid_)
  {
    // Calculate timing
    auto previous = trajectory_.begin();
    auto it = previous;
    it->time_ = 0.0;
    ++it;
    // Each step's path acceleration is constant. It is read off the change in velocity over the step's own duration,
    // which stays accurate on short steps where the change in position cancels. A step too short for that takes it
    // from a neighbour on its side of any segment end: the next step if it starts on one, else the previous one
    const double min_step_duration = 1e-6 * time_step_;
    std::vector<double> segment_ends;
    for (const auto& switching_point : path_.getSwitchingPoints())
    {
      if (switching_point.second)
        segment_ends.push_back(switching_point.first);
    }
    const auto starts_on_segment_end = [&segment_ends](double path_pos) {
      return std::binary_search(segment_ends.begin(), segment_ends.end(), path_pos);
    };

    std::vector<std::list<TrajectoryStep>::iterator> short_steps;
    while (it != trajectory_.end())
    {
      const double step_duration =
          (it->path_pos_ - previous->path_pos_) / ((it->path_vel_ + previous->path_vel_) / 2.0);
      it->time_ = previous->time_ + step_duration;
      if (step_duration > min_step_duration)
        it->path_acc_ = (it->path_vel_ - previous->path_vel_) / step_duration;
      else
        short_steps.push_back(it);
      previous = it;
      ++it;
    }

    // An acceleration taken from a step that is itself short would be just as inaccurate
    const auto is_short = [&short_steps](std::list<TrajectoryStep>::iterator step) {
      return std::find(short_steps.begin(), short_steps.end(), step) != short_steps.end();
    };
    for (auto step : short_steps)
    {
      auto before = step;
      while (before != std::next(trajectory_.begin()) && is_short(before))
        --before;
      auto after = step;
      while (std::next(after) != trajectory_.end() && is_short(after))
        ++after;
      const bool before_usable = !is_short(before);
      const bool after_usable = !is_short(after);
      const bool prefer_after = starts_on_segment_end(std::prev(step)->path_pos_);
      if (after_usable && (prefer_after || !before_usable))
        step->path_acc_ = after->path_acc_;
      else if (before_usable)
        step->path_acc_ = before->path_acc_;
    }

    // A stretch of zero path velocity, such as a zero velocity limit on a moving joint gives, is timed as infinite
    if (!std::isfinite(trajectory_.back().time_))
    {
      valid_ = false;
      TESSERACT_LOG_ERROR("Trajectory duration is not finite, a moving joint likely has a zero velocity limit");
    }
  }
}

// Returns true if end of path is reached.
bool Trajectory::getNextSwitchingPoint(double path_pos,
                                       TrajectoryStep& next_switching_point,
                                       double& before_acceleration,
                                       double& after_acceleration)
{
  TrajectoryStep acceleration_switching_point(path_pos, 0.0);
  double acceleration_before_acceleration{ NAN };
  double acceleration_after_acceleration{ NAN };
  bool acceleration_reached_end{ false };
  do  // NOLINT(cppcoreguidelines-avoid-do-while)
  {
    acceleration_reached_end = getNextAccelerationSwitchingPoint(acceleration_switching_point.path_pos_,
                                                                 acceleration_switching_point,
                                                                 acceleration_before_acceleration,
                                                                 acceleration_after_acceleration);
  } while (!acceleration_reached_end &&
           acceleration_switching_point.path_vel_ > getVelocityMaxPathVelocity(acceleration_switching_point.path_pos_));

  TrajectoryStep velocity_switching_point(path_pos, 0.0);
  double velocity_before_acceleration{ NAN };
  double velocity_after_acceleration{ NAN };
  bool velocity_reached_end{ false };
  do  // NOLINT(cppcoreguidelines-avoid-do-while)
  {
    velocity_reached_end = getNextVelocitySwitchingPoint(velocity_switching_point.path_pos_,
                                                         velocity_switching_point,
                                                         velocity_before_acceleration,
                                                         velocity_after_acceleration);
  } while (
      !velocity_reached_end && velocity_switching_point.path_pos_ <= acceleration_switching_point.path_pos_ &&
      (velocity_switching_point.path_vel_ > getAccelerationMaxPathVelocity(velocity_switching_point.path_pos_ - EPS) ||
       velocity_switching_point.path_vel_ > getAccelerationMaxPathVelocity(velocity_switching_point.path_pos_ + EPS)));

  if (acceleration_reached_end && velocity_reached_end)
  {
    return true;
  }

  if (!acceleration_reached_end &&
      (velocity_reached_end || acceleration_switching_point.path_pos_ <= velocity_switching_point.path_pos_))
  {
    next_switching_point = acceleration_switching_point;
    before_acceleration = acceleration_before_acceleration;
    after_acceleration = acceleration_after_acceleration;
    return false;
  }

  next_switching_point = velocity_switching_point;
  before_acceleration = velocity_before_acceleration;
  after_acceleration = velocity_after_acceleration;
  return false;
}

bool Trajectory::getNextAccelerationSwitchingPoint(double path_pos,
                                                   TrajectoryStep& next_switching_point,
                                                   double& before_acceleration,
                                                   double& after_acceleration)
{
  double switching_path_pos = path_pos;
  double switching_path_vel{ NAN };
  while (true)
  {
    bool discontinuity{ false };
    switching_path_pos = path_.getNextSwitchingPoint(switching_path_pos, discontinuity);

    if (switching_path_pos > path_.getLength() - EPS)
    {
      return true;
    }

    if (discontinuity)
    {
      const double before_path_vel = getAccelerationMaxPathVelocity(switching_path_pos - EPS);
      const double after_path_vel = getAccelerationMaxPathVelocity(switching_path_pos + EPS);
      switching_path_vel = std::min(before_path_vel, after_path_vel);
      before_acceleration = getMinMaxPathAcceleration(switching_path_pos - EPS, switching_path_vel, false);
      after_acceleration = getMinMaxPathAcceleration(switching_path_pos + EPS, switching_path_vel, true);

      if ((before_path_vel > after_path_vel ||
           getMinMaxPhaseSlope(switching_path_pos - EPS, switching_path_vel, false) >
               getAccelerationMaxPathVelocityDeriv(switching_path_pos - (2.0 * EPS))) &&
          (before_path_vel < after_path_vel ||
           getMinMaxPhaseSlope(switching_path_pos + EPS, switching_path_vel, true) <
               getAccelerationMaxPathVelocityDeriv(switching_path_pos + (2.0 * EPS))))
      {
        break;
      }
    }
    else
    {
      switching_path_vel = getAccelerationMaxPathVelocity(switching_path_pos);
      before_acceleration = 0.0;
      after_acceleration = 0.0;

      if (getAccelerationMaxPathVelocityDeriv(switching_path_pos - EPS) < 0.0 &&
          getAccelerationMaxPathVelocityDeriv(switching_path_pos + EPS) > 0.0)
      {
        break;
      }
    }
  }

  next_switching_point = TrajectoryStep(switching_path_pos, switching_path_vel);
  return false;
}

bool Trajectory::getNextVelocitySwitchingPoint(double path_pos,
                                               TrajectoryStep& next_switching_point,
                                               double& before_acceleration,
                                               double& after_acceleration)
{
  const double step_size = 0.001;
  const double accuracy = 0.000001;

  // Start the scan one step past path_pos, so the switching point it brackets lies past path_pos and a search that
  // resumes from it advances. Where the two slopes agree to rounding, as under very high velocity limits, path_pos
  // itself could otherwise stop the scan and bracket a switching point behind it
  bool start = getMinMaxPhaseSlope(path_pos, getVelocityMaxPathVelocity(path_pos), false) >=
               getVelocityMaxPathVelocityDeriv(path_pos);
  do  // NOLINT(cppcoreguidelines-avoid-do-while)
  {
    path_pos += step_size;

    if (getMinMaxPhaseSlope(path_pos, getVelocityMaxPathVelocity(path_pos), false) >=
        getVelocityMaxPathVelocityDeriv(path_pos))
    {
      start = true;
    }
  } while ((!start || getMinMaxPhaseSlope(path_pos, getVelocityMaxPathVelocity(path_pos), false) >
                          getVelocityMaxPathVelocityDeriv(path_pos)) &&
           path_pos < path_.getLength());

  if (path_pos >= path_.getLength())
  {
    return true;  // end of trajectory reached
  }

  double before_path_pos = path_pos - step_size;
  double after_path_pos = path_pos;
  while (after_path_pos - before_path_pos > accuracy)
  {
    path_pos = (before_path_pos + after_path_pos) / 2.0;
    if (getMinMaxPhaseSlope(path_pos, getVelocityMaxPathVelocity(path_pos), false) >
        getVelocityMaxPathVelocityDeriv(path_pos))
    {
      before_path_pos = path_pos;
    }
    else
    {
      after_path_pos = path_pos;
    }
  }

  before_acceleration = getMinMaxPathAcceleration(before_path_pos, getVelocityMaxPathVelocity(before_path_pos), false);
  after_acceleration = getMinMaxPathAcceleration(after_path_pos, getVelocityMaxPathVelocity(after_path_pos), true);
  next_switching_point = TrajectoryStep(after_path_pos, getVelocityMaxPathVelocity(after_path_pos));
  return false;
}

// Returns true if end of path is reached
bool Trajectory::integrateForward(std::list<TrajectoryStep>& trajectory, double acceleration)
{
  double path_pos = trajectory.back().path_pos_;
  double path_vel = trajectory.back().path_vel_;

  const std::list<std::pair<double, bool>>& switching_points = path_.getSwitchingPoints();
  auto next_discontinuity = switching_points.begin();

  while (true)
  {
    while ((next_discontinuity != switching_points.end()) &&
           (next_discontinuity->first <= path_pos || !next_discontinuity->second))
    {
      ++next_discontinuity;
    }

    double old_path_pos = path_pos;
    double old_path_vel = path_vel;

    path_vel += time_step_ * acceleration;
    path_pos += time_step_ * 0.5 * (old_path_vel + path_vel);

    // Every step that crosses a discontinuity ends on it, so none goes on with the acceleration of the segment before,
    // and none ends just past it, which would add an almost identical step in the next run
    // (https://github.com/ros-planning/moveit/issues/1665)
    if (next_discontinuity != switching_points.end() && path_pos > next_discontinuity->first)
    {
      path_vel = old_path_vel +
                 (next_discontinuity->first - old_path_pos) * (path_vel - old_path_vel) / (path_pos - old_path_pos);
      path_pos = next_discontinuity->first;
    }

    if (path_pos > path_.getLength())
    {
      trajectory.emplace_back(path_pos, path_vel);
      return true;
    }

    if (path_vel < 0.0)
    {
      valid_ = false;
      TESSERACT_LOG_ERROR("Error while integrating forward: Negative path velocity");
      return true;
    }

    if (path_vel > getVelocityMaxPathVelocity(path_pos) &&
        getMinMaxPhaseSlope(old_path_pos, getVelocityMaxPathVelocity(old_path_pos), false) <=
            getVelocityMaxPathVelocityDeriv(old_path_pos))
    {
      path_vel = getVelocityMaxPathVelocity(path_pos);
    }

    trajectory.emplace_back(path_pos, path_vel);
    acceleration = getMinMaxPathAcceleration(path_pos, path_vel, true);

    // At rest with no acceleration the position never changes and nothing else ends the loop
    if (path_vel == 0 && acceleration == 0)
    {
      valid_ = false;
      TESSERACT_LOG_ERROR("Error while integrating forward: zero path velocity and acceleration, a moving joint "
                          "likely has a zero acceleration limit");
      return true;
    }

    if (path_vel > getAccelerationMaxPathVelocity(path_pos) || path_vel > getVelocityMaxPathVelocity(path_pos))
    {
      // Find more accurate intersection with max-velocity curve using bisection
      TrajectoryStep overshoot = trajectory.back();
      trajectory.pop_back();
      double before = trajectory.back().path_pos_;
      double before_path_vel = trajectory.back().path_vel_;
      double after = overshoot.path_pos_;
      double after_path_vel = overshoot.path_vel_;
      while (after - before > EPS)
      {
        const double midpoint = 0.5 * (before + after);
        double midpoint_path_vel = 0.5 * (before_path_vel + after_path_vel);

        if (midpoint_path_vel > getVelocityMaxPathVelocity(midpoint) &&
            getMinMaxPhaseSlope(before, getVelocityMaxPathVelocity(before), false) <=
                getVelocityMaxPathVelocityDeriv(before))
        {
          midpoint_path_vel = getVelocityMaxPathVelocity(midpoint);
        }

        if (midpoint_path_vel > getAccelerationMaxPathVelocity(midpoint) ||
            midpoint_path_vel > getVelocityMaxPathVelocity(midpoint))
        {
          after = midpoint;
          after_path_vel = midpoint_path_vel;
        }
        else
        {
          before = midpoint;
          before_path_vel = midpoint_path_vel;
        }
      }
      trajectory.emplace_back(before, before_path_vel);

      if (getAccelerationMaxPathVelocity(after) < getVelocityMaxPathVelocity(after))
      {
        // Steps that cross a discontinuity end on it, so reaching it counts as passing it. Past the last discontinuity
        // only the end of the path follows, which after cannot pass
        if (next_discontinuity != switching_points.end() && after >= next_discontinuity->first)
        {
          return false;
        }

        if (getMinMaxPhaseSlope(trajectory.back().path_pos_, trajectory.back().path_vel_, true) >
            getAccelerationMaxPathVelocityDeriv(trajectory.back().path_pos_))
        {
          return false;
        }
      }
      else
      {
        if (getMinMaxPhaseSlope(trajectory.back().path_pos_, trajectory.back().path_vel_, false) >
            getVelocityMaxPathVelocityDeriv(trajectory.back().path_pos_))
        {
          return false;
        }
      }
    }
  }
}

void Trajectory::integrateBackward(std::list<TrajectoryStep>& start_trajectory,
                                   double path_pos,
                                   double path_vel,
                                   double acceleration)
{
  auto start2 = start_trajectory.end();
  --start2;
  auto start1 = start2;
  --start1;
  std::list<TrajectoryStep> trajectory;
  double slope{ 0 };
  assert(start1->path_pos_ < path_pos ||
         tesseract::common::almostEqualRelativeAndAbs(start1->path_pos_, path_pos, EPS));

  // The segment ends, from the last one before path_pos down
  const std::list<std::pair<double, bool>>& switching_points = path_.getSwitchingPoints();
  auto prev_discontinuity = switching_points.rbegin();

  while (start1 != start_trajectory.begin() || path_pos >= 0.0)
  {
    // Take a backward step while the forward step starts below the backward one's lower end, else go to the forward
    // step before. Strict, so that no overlapping pair of steps is skipped. The first step is taken regardless, as the
    // pass may start up to EPS below the forward step's start and the meeting test needs a backward step, and so is
    // every step once the forward steps run out
    if (trajectory.empty() || start1->path_pos_ < path_pos || start1 == start_trajectory.begin())
    {
      trajectory.emplace_front(path_pos, path_vel);
      const double old_path_pos = path_pos;
      const double old_path_vel = path_vel;
      path_vel -= time_step_ * acceleration;
      path_pos -= time_step_ * 0.5 * (path_vel + trajectory.front().path_vel_);

      // A step must not cross a segment end, where the curvature, and with it the feasible acceleration, jumps: end it
      // there, as integrateForward does, and take the next step's acceleration from the segment it lies in
      while (prev_discontinuity != switching_points.rend() &&
             (prev_discontinuity->first >= old_path_pos || !prev_discontinuity->second))
        ++prev_discontinuity;
      // Taken from the whole step: the clipped one lies on the same line, and can be too short to give it accurately
      slope = (trajectory.front().path_vel_ - path_vel) / (trajectory.front().path_pos_ - path_pos);

      // A step that lands on the segment end takes the next acceleration below it too
      if (prev_discontinuity != switching_points.rend() && path_pos <= prev_discontinuity->first)
      {
        path_vel = old_path_vel +
                   (prev_discontinuity->first - old_path_pos) * (path_vel - old_path_vel) / (path_pos - old_path_pos);
        path_pos = prev_discontinuity->first;
        acceleration = getMinMaxPathAcceleration(
            std::nextafter(path_pos, -std::numeric_limits<double>::infinity()), path_vel, false);
      }
      else
      {
        acceleration = getMinMaxPathAcceleration(path_pos, path_vel, false);
      }

      // At rest with no deceleration the position never changes and nothing else ends the loop
      if (path_vel == 0 && acceleration == 0)
      {
        valid_ = false;
        TESSERACT_LOG_ERROR("Error while integrating backward: zero path velocity and acceleration");
        end_trajectory_ = trajectory;
        return;
      }

      if (path_vel < 0.0)
      {
        valid_ = false;
        TESSERACT_LOG_ERROR("Error while integrating backward: Negative path velocity");
        end_trajectory_ = trajectory;
        return;
      }
    }
    else
    {
      --start1;
      --start2;
    }

    // Check for intersection between current start trajectory and backward
    // trajectory segments
    const double start_slope = (start2->path_vel_ - start1->path_vel_) / (start2->path_pos_ - start1->path_pos_);

    // Parallel lines meet only where they coincide, and then anywhere along the step. Otherwise the intersection is
    // left NaN, which no position test below accepts; only the comparison at the overlap's ends can still join such
    // lines
    double intersection_path_pos = std::numeric_limits<double>::quiet_NaN();
    bool coincident{ false };
    if (!tesseract::common::almostEqualRelativeAndAbs(slope, start_slope, EPS))
    {
      intersection_path_pos =
          (start1->path_vel_ - path_vel + slope * path_pos - start_slope * start1->path_pos_) / (slope - start_slope);
    }
    else if (tesseract::common::almostEqualRelativeAndAbs(
                 start1->path_vel_ + (start_slope * (path_pos - start1->path_pos_)), path_vel, EPS))
    {
      coincident = true;
      intersection_path_pos = start1->path_pos_ + (start2->path_pos_ - start1->path_pos_) / 2.0;
    }

    // The meeting point must lie on both steps
    const double pos_max = std::max(start1->path_pos_, path_pos);
    const double pos_min = std::min(start2->path_pos_, trajectory.front().path_pos_);
    bool meets = pos_max <= intersection_path_pos && intersection_path_pos <= pos_min;

    // The forward pass ends up to EPS short of where it meets the limit curve, as its bisection stops there, so past
    // its last step the backward step may meet the forward one's line within EPS
    if (!meets && !coincident && std::next(start2) == start_trajectory.end() &&
        start2->path_pos_ < intersection_path_pos && intersection_path_pos <= start2->path_pos_ + EPS &&
        path_pos <= intersection_path_pos)
      meets = true;

    // Lines that cross on the overlap of the steps can round their meeting point just outside it. Then read each step
    // at the overlap's ends, exactly at its own ends, so that two pairs of steps sharing an end agree on which side of
    // the forward step the backward one passes, and join where the difference vanishes
    if (!meets && !coincident && pos_max <= pos_min)
    {
      const auto on = [](double pos0, double vel0, double pos1, double vel1, double pos) {
        if (pos == pos0)
          return vel0;
        if (pos == pos1)
          return vel1;
        return vel0 + ((vel1 - vel0) * (pos - pos0) / (pos1 - pos0));
      };
      const double front_pos = trajectory.front().path_pos_;
      const double front_vel = trajectory.front().path_vel_;
      const double low = on(path_pos, path_vel, front_pos, front_vel, pos_max) -
                         on(start1->path_pos_, start1->path_vel_, start2->path_pos_, start2->path_vel_, pos_max);
      const double high = on(path_pos, path_vel, front_pos, front_vel, pos_min) -
                          on(start1->path_pos_, start1->path_vel_, start2->path_pos_, start2->path_vel_, pos_min);
      if ((low >= 0.0 && high <= 0.0) || (low <= 0.0 && high >= 0.0))
      {
        meets = true;
        intersection_path_pos = (low == high) ?
                                    pos_min :
                                    std::clamp(pos_max + ((pos_min - pos_max) * low / (low - high)), pos_max, pos_min);
      }
    }

    if (meets)
    {
      // Coincident lines can still lie up to EPS apart, so join them on the backward line, which the next step follows
      const double intersection_path_vel =
          coincident ? path_vel + (slope * (intersection_path_pos - path_pos)) :
                       start1->path_vel_ + (start_slope * (intersection_path_pos - start1->path_pos_));

      // Join the two at the meeting point, dropping forward steps the backward pass already starts below, so that
      // positions, and the times computed from them, never go back. A meeting point past the backward pass's start,
      // which the forward pass's end allows, or within a negligible fraction of the gap from either neighbour, joins at
      // that neighbour instead, so no step lasts only a rounding error
      start_trajectory.erase(start2, start_trajectory.end());
      while (start_trajectory.size() > 1 && !(start_trajectory.back().path_pos_ < trajectory.front().path_pos_))
        start_trajectory.pop_back();
      const double gap = trajectory.front().path_pos_ - start_trajectory.back().path_pos_;
      constexpr double negligible = 1e-12;
      if (intersection_path_pos - start_trajectory.back().path_pos_ > negligible * gap &&
          trajectory.front().path_pos_ - intersection_path_pos > negligible * gap)
        start_trajectory.emplace_back(intersection_path_pos, intersection_path_vel);
      start_trajectory.splice(start_trajectory.end(), trajectory);
      return;
    }
  }

  valid_ = false;
  if (start_trajectory.front().path_vel_ > 0.0)
    TESSERACT_LOG_ERROR("Error while integrating backward: Did not hit start trajectory; the path leaves no room to "
                        "slow down from the start velocity");
  else
    TESSERACT_LOG_ERROR("Error while integrating backward: Did not hit start trajectory");
  end_trajectory_ = trajectory;
}

double Trajectory::getMinMaxPathAcceleration(double path_position, double path_velocity, bool max)
{
  Eigen::VectorXd config_deriv = path_.getTangent(path_position);
  Eigen::VectorXd config_deriv2 = path_.getCurvature(path_position);
  double factor = max ? 1.0 : -1.0;
  double max_path_acceleration = std::numeric_limits<double>::max();
  for (unsigned int i = 0; i < joint_num_; ++i)
  {
    if (!tesseract::common::almostEqualRelativeAndAbs(config_deriv[i], 0.0, std::numeric_limits<double>::epsilon()))
    {
      max_path_acceleration =
          std::min(max_path_acceleration,
                   (max_acceleration_[i] / std::abs(config_deriv[i])) -
                       (factor * config_deriv2[i] * path_velocity * path_velocity / config_deriv[i]));
    }
  }
  return factor * max_path_acceleration;
}

double Trajectory::getMinMaxPhaseSlope(double path_position, double path_velocity, bool max)
{
  return getMinMaxPathAcceleration(path_position, path_velocity, max) / path_velocity;
}

double Trajectory::getAccelerationMaxPathVelocity(double path_pos) const
{
  double max_path_velocity = std::numeric_limits<double>::infinity();
  const Eigen::VectorXd config_deriv = path_.getTangent(path_pos);
  const Eigen::VectorXd config_deriv2 = path_.getCurvature(path_pos);
  for (unsigned int i = 0; i < joint_num_; ++i)
  {
    if (config_deriv[i] != 0.0)
    {
      for (unsigned int j = i + 1; j < joint_num_; ++j)
      {
        if (config_deriv[j] != 0.0)
        {
          double a_ij = (config_deriv2[i] / config_deriv[i]) - (config_deriv2[j] / config_deriv[j]);
          if (a_ij != 0.0)
          {
            max_path_velocity = std::min(max_path_velocity,
                                         sqrt((max_acceleration_[i] / std::abs(config_deriv[i]) +
                                               max_acceleration_[j] / std::abs(config_deriv[j])) /
                                              std::abs(a_ij)));
          }
        }
      }
    }
    else if (config_deriv2[i] != 0.0)
    {
      max_path_velocity = std::min(max_path_velocity, sqrt(max_acceleration_[i] / std::abs(config_deriv2[i])));
    }
  }
  return max_path_velocity;
}

double Trajectory::getVelocityMaxPathVelocity(double path_pos) const
{
  const Eigen::VectorXd tangent = path_.getTangent(path_pos);
  double max_path_velocity = std::numeric_limits<double>::max();
  for (unsigned int i = 0; i < joint_num_; ++i)
  {
    max_path_velocity = std::min(max_path_velocity, max_velocity_[i] / std::abs(tangent[i]));
  }
  return max_path_velocity;
}

double Trajectory::getAccelerationMaxPathVelocityDeriv(double path_pos)
{
  return (getAccelerationMaxPathVelocity(path_pos + EPS) - getAccelerationMaxPathVelocity(path_pos - EPS)) /
         (2.0 * EPS);
}

double Trajectory::getVelocityMaxPathVelocityDeriv(double path_pos)
{
  const Eigen::VectorXd tangent = path_.getTangent(path_pos);
  double max_path_velocity = std::numeric_limits<double>::max();
  unsigned int active_constraint{ 0 };
  for (unsigned int i = 0; i < joint_num_; ++i)
  {
    const double this_max_path_velocity = max_velocity_[i] / std::abs(tangent[i]);
    if (this_max_path_velocity < max_path_velocity)
    {
      max_path_velocity = this_max_path_velocity;
      active_constraint = i;
    }
  }
  return -(max_velocity_[active_constraint] * path_.getCurvature(path_pos)[active_constraint]) /
         (tangent[active_constraint] * std::abs(tangent[active_constraint]));
}

bool Trajectory::isValid() const { return valid_; }

double Trajectory::getDuration() const { return trajectory_.back().time_; }

bool Trajectory::assignData(InstructionsTrajectory& trajectory, const std::vector<std::size_t>& kept) const
{
  return assignData(trajectory, kept, 0.0);
}

bool Trajectory::assignData(InstructionsTrajectory& trajectory,
                            const std::vector<std::size_t>& kept,
                            double time_offset) const
{
  const auto& dist_mapping = path_.getMapping();
  if (kept.size() != dist_mapping.size() || kept.size() < 2)
  {
    TESSERACT_LOG_ERROR(
        "TimeOptimalTrajectoryGeneration, {} kept waypoints for a path of {} points", kept.size(), dist_mapping.size());
    return false;
  }
  for (std::size_t j = 0; j < kept.size(); ++j)
  {
    if (kept[j] >= static_cast<std::size_t>(trajectory.size()) || (j > 0 && kept[j] <= kept[j - 1]))
    {
      TESSERACT_LOG_ERROR("TimeOptimalTrajectoryGeneration, kept waypoint indices must increase within the trajectory");
      return false;
    }
  }

  // Time every point before writing any, so a failure leaves the waypoints as they were
  std::vector<double> times(kept.size());
  std::vector<PathData> path_data(kept.size());
  for (std::size_t j = 0; j < kept.size(); ++j)
  {
    // The first point is at the start and the last at the end, so only the points between are looked up
    double time = 0.0;
    if (j + 1 == kept.size())
      time = getDuration();
    else if (j > 0)
      time = getTime(dist_mapping[j]);
    path_data[j] = getPathData(time);
    times[j] = time_offset + time;

    // Written as a negation so a NaN time fails too
    if (j > 0 && !(times[j] > times[j - 1]))
    {
      TESSERACT_LOG_ERROR("TimeOptimalTrajectoryGeneration, waypoint {} is timed at {}, not after the previous one at "
                          "{}",
                          kept[j],
                          times[j],
                          times[j - 1]);
      return false;
    }
  }

  // The path ends at rest at the last waypoint
  path_data.back().path_vel = 0.0;

  for (std::size_t j = 0; j < kept.size(); ++j)
  {
    const auto i = static_cast<Eigen::Index>(kept[j]);
    trajectory.setData(i, getVelocity(path_data[j]), getAcceleration(path_data[j]), times[j]);

    // The path passes a blended corner at its mapped point, not at the corner itself
    if (path_.isBlended(j))
      trajectory.getPosition(i) = path_.getConfig(dist_mapping[j]);
  }
  return true;
}

std::list<Trajectory::TrajectoryStep>::const_iterator Trajectory::getTrajectorySegment(double time) const
{
  if (time >= trajectory_.back().time_)
  {
    auto last = trajectory_.end();
    last--;
    return last;
  }

  // Restart unless time went forward; written as a negation so a NaN time restarts too, and so does any time after one
  // Any time from 0 on is past the first step, so starting after it changes nothing there, and a time before 0 or a NaN
  // gets the first interval rather than none
  if (!(time >= lookup_.time))
  {
    lookup_.time_segment = std::next(trajectory_.begin());
  }
  while (time >= lookup_.time_segment->time_)
  {
    ++lookup_.time_segment;
  }
  lookup_.time = time;
  return lookup_.time_segment;
}

std::list<Trajectory::TrajectoryStep>::const_iterator Trajectory::getTrajectorySegmentFromDist(double pos) const
{
  if (pos >= trajectory_.back().path_pos_)
  {
    auto last = trajectory_.end();
    last--;
    return last;
  }

  if (pos < 0)
    return std::next(trajectory_.begin());

  // Find the first step past pos, resuming from the last one found unless pos went back. No step before that one is
  // past the last pos, so a scan from the first step reaches it too, however the steps are ordered
  auto it = (pos >= lookup_.path_pos) ? lookup_.path_pos_segment : trajectory_.begin();
  for (; it != trajectory_.end(); ++it)
  {
    if (pos < it->path_pos_)
    {
      lookup_.path_pos = pos;
      lookup_.path_pos_segment = it;
      return it;
    }
  }

  throw std::runtime_error("Failed to find trajectory segment, this should not happen");
}

PathData Trajectory::getPathData(double time) const
{
  PathData data;

  auto it = getTrajectorySegment(time);
  auto previous = it;
  previous--;

  const double elapsed = time - previous->time_;
  data.path_acc = it->path_acc_;
  data.path_pos = previous->path_pos_ + (elapsed * previous->path_vel_) + (0.5 * elapsed * elapsed * data.path_acc);
  data.path_vel = previous->path_vel_ + (elapsed * data.path_acc);

  // Keep the position on the step, below its end, so the path is read in the segment the step lies in
  if (it->path_pos_ > previous->path_pos_)
    data.path_pos = std::min(std::max(data.path_pos, previous->path_pos_),
                             std::nextafter(it->path_pos_, -std::numeric_limits<double>::infinity()));
  data.time = time;
  data.prev_path_pos = previous->path_pos_;
  data.prev_path_vel = previous->path_vel_;
  data.prev_time = previous->time_;
  return data;
}

double Trajectory::getTime(double pos) const
{
  auto it = getTrajectorySegmentFromDist(pos);
  assert(it != trajectory_.begin());
  auto previous = it;
  previous--;

  assert(pos >= previous->path_pos_ && pos <= it->path_pos_);

  const double time_step = it->time_ - previous->time_;
  const double acceleration = it->path_acc_;

  // Solve 0.5 * acceleration * dt^2 + velocity * dt = distance for the root that is not negative, in the form that does
  // not cancel: it holds for zero acceleration, where the textbook form divides zero by zero, and for zero velocity
  const double distance = pos - previous->path_pos_;
  const double velocity = previous->path_vel_;
  const double discriminant = std::max(0.0, (velocity * velocity) + (2.0 * acceleration * distance));
  const double denominator = velocity + std::sqrt(discriminant);

  // Only at rest, within rounding of the step's start
  if (!(denominator > 0.0))
    return previous->time_;

  // Rounding can put the root just outside the step; std::max and std::min keep a NaN
  const double dt = std::min(std::max(2.0 * distance / denominator, 0.0), time_step);
  return (previous->time_ + dt);
}

Eigen::VectorXd Trajectory::getPosition(const PathData& data) const { return path_.getConfig(data.path_pos); }

Eigen::VectorXd Trajectory::getVelocity(const PathData& data) const
{
  return path_.getTangent(data.path_pos) * data.path_vel;
}

Eigen::VectorXd Trajectory::getAcceleration(const PathData& data) const
{
  // The joint acceleration along the path, d2q/dt2 = q'(s) s'' + q''(s) s'^2, which the integration bounds
  return (path_.getTangent(data.path_pos) * data.path_acc) +
         (path_.getCurvature(data.path_pos) * data.path_vel * data.path_vel);
}
}  // namespace totg
}  // namespace tesseract::time_parameterization
