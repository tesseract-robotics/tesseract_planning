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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <limits>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <vector>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_time_parameterization/totg/time_optimal_trajectory_generation.h>
#include <tesseract_time_parameterization/totg/time_optimal_trajectory_generation_profiles.h>
#include <tesseract_time_parameterization/core/instructions_trajectory.h>
#include <tesseract_common/kinematic_limits.h>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/profile_dictionary.h>
#include <tesseract_common/utils.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_environment/environment.h>

constexpr double EPS = 0.000001;

namespace tesseract_planning
{
TimeOptimalTrajectoryGeneration::TimeOptimalTrajectoryGeneration(std::string name)
  : TimeParameterization(std::move(name))
{
}

bool TimeOptimalTrajectoryGeneration::compute(CompositeInstruction& composite_instruction,
                                              const tesseract_environment::Environment& env,
                                              const tesseract_common::ProfileDictionary& profiles) const
{
  auto flattened = composite_instruction.flatten(moveFilter);
  if (flattened.empty())
    return true;

  const tesseract_common::ManipulatorInfo manip_info = composite_instruction.getManipulatorInfo();
  auto jg = env.getJointGroup(manip_info.manipulator);
  tesseract_common::KinematicLimits limits = jg->getLimits();
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
    throw std::runtime_error("IterativeSplineParameterization, velocity scale factor must be greater than zero!");

  // NOLINTNEXTLINE(readability-simplify-boolean-expr)
  if (!((ci_profile->max_acceleration_scaling_factor > 0.0) && (ci_profile->max_acceleration_scaling_factor <= 1.0)))
    throw std::runtime_error("IterativeSplineParameterization, velocity scale factor must be greater than zero!");

  // Validate limits
  if (velocity_limits.rows() != acceleration_limits.rows())
  {
    CONSOLE_BRIDGE_logError("Invalid velocity or acceleration specified. They should be the same length");
  }

  InstructionsTrajectory trajectory(flattened);
  const Eigen::Index num_joints = jg->numJoints();
  auto num_points = static_cast<std::size_t>(trajectory.size());

  // This lib does not actually work properly when angles wrap around, so we need to unwind the path first
  //  trajectory.unwind(); /// @todo

  // Have to convert into Eigen data structs and remove repeated points
  //  (https://github.com/tobiaskunz/trajectories/issues/3)
  std::list<Eigen::VectorXd> points;
  std::vector<std::size_t> mapping;
  for (Eigen::Index p = 0; p < static_cast<Eigen::Index>(num_points); ++p)
  {
    const Eigen::VectorXd& position = trajectory.getPosition(p);
    bool diverse_point = (p == 0);

    if (p > 0)
    {
      const Eigen::VectorXd& prev_point = points.back();
      for (Eigen::Index j = 0; j < num_joints; j++)
      {
        if (std::abs(position[j] - prev_point[j]) > ci_profile->min_angle_change)
          diverse_point = true;
      }
    }

    if (diverse_point)
      points.push_back(position);

    // Need to store the index mapping for assignData
    mapping.push_back(points.size() - 1);
  }

  // Return trajectory with only the first waypoint if there are not multiple diverse points
  if (points.size() == 1)
  {
    CONSOLE_BRIDGE_logDebug("Trajectory is parameterized with 0.0 dynamics since it only contains a single distinct "
                            "waypoint.");

    // Set velocity, acceleration and time to zero for all points in the trajectory.
    for (long i = 0; i < trajectory.size(); ++i)
      trajectory.setData(i, Eigen::VectorXd::Zero(num_joints), Eigen::VectorXd::Zero(num_joints), 0);

    return true;
  }

  // Append a dummy joint as a workaround to https://github.com/ros-industrial-consortium/tesseract_planning/issues/27
  std::list<Eigen::VectorXd> new_points;
  double dummy = 1.0;
  for (auto& point : points)
  {
    Eigen::VectorXd new_point(point.size() + 1);
    new_point << point, dummy;
    new_points.push_back(new_point);
    dummy += 1.0;
  }

  Eigen::VectorXd max_velocity_dummy_appended(velocity_limits.rows() + 1);
  max_velocity_dummy_appended << (velocity_limits.col(1) * ci_profile->max_velocity_scaling_factor),
      std::numeric_limits<double>::max();
  Eigen::VectorXd max_acceleration_dummy_appended(acceleration_limits.rows() + 1);
  max_acceleration_dummy_appended << (acceleration_limits.col(1) * ci_profile->max_acceleration_scaling_factor),
      std::numeric_limits<double>::max();

  // Now actually call the algorithm
  totg::Path path(new_points, ci_profile->path_tolerance);
  totg::Trajectory parameterized(path, max_velocity_dummy_appended, max_acceleration_dummy_appended, 0.001);
  if (!parameterized.isValid())
  {
    CONSOLE_BRIDGE_logError("Unable to parameterize trajectory.");
    return false;
  }

  return parameterized.assignData(trajectory, mapping);
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
    return;
  auto path_iterator1 = path.begin();
  auto path_iterator2 = path_iterator1;
  ++path_iterator2;
  std::list<Eigen::VectorXd>::const_iterator path_iterator3;
  Eigen::VectorXd start_config = *path_iterator1;
  mapping_.reserve(path.size());
  mapping_.push_back(0);
  double l{ 0 };
  while (path_iterator2 != path.end())
  {
    path_iterator3 = path_iterator2;
    ++path_iterator3;
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
      l += blend_segment->getLength();
      path_segments_.push_back(std::move(blend_segment));
    }
    else
    {
      path_segments_.push_back(std::make_unique<LinearPathSegment>(start_config, *path_iterator2));
      l += path_segments_.back()->getLength();
      mapping_.push_back(l);
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
}

Path::Path(const Path& path) : length_(path.length_), mapping_(path.mapping_), switching_points_(path.switching_points_)
{
  for (const std::unique_ptr<PathSegment>& path_segment : path.path_segments_)
    path_segments_.emplace_back(path_segment->clone());
}

double Path::getLength() const { return length_; }

const std::vector<double>& Path::getMapping() const { return mapping_; }

PathSegment* Path::getPathSegment(double& s) const
{
  auto it = path_segments_.begin();
  auto next = it;
  ++next;
  while (next != path_segments_.end() && s >= (*next)->position_)
  {
    it = next;
    ++next;
  }
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

std::list<std::pair<double, bool>> Path::getSwitchingPoints() const { return switching_points_; }

Trajectory::Trajectory(const Path& path,
                       const Eigen::VectorXd& max_velocity,
                       const Eigen::VectorXd& max_acceleration,  // NOLINT
                       double time_step)
  : path_(path)
  , max_velocity_(max_velocity)
  , max_acceleration_(max_acceleration)
  , joint_num_(max_velocity.size())
  , time_step_(time_step)
  , cached_time_(std::numeric_limits<double>::max())
{
  trajectory_.emplace_back(0.0, 0.0);
  double after_acceleration = getMinMaxPathAcceleration(0.0, 0.0, true);
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
    while (it != trajectory_.end())
    {
      it->time_ =
          previous->time_ + (it->path_pos_ - previous->path_pos_) / ((it->path_vel_ + previous->path_vel_) / 2.0);
      previous = it;
      ++it;
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

  bool start = false;
  path_pos -= step_size;
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

  std::list<std::pair<double, bool>> switching_points = path_.getSwitchingPoints();
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

    if (next_discontinuity != switching_points.end() && path_pos > next_discontinuity->first)
    {
      // Avoid having a TrajectoryStep with path_pos near a switching point which will cause an almost identical
      // TrajectoryStep get added in the next run (https://github.com/ros-planning/moveit/issues/1665)
      if (path_pos - next_discontinuity->first < EPS)
      {
        continue;
      }
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
      CONSOLE_BRIDGE_logError("Error while integrating forward: Negative path velocity");
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
        if (after > next_discontinuity->first)
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
        if (getMinMaxPhaseSlope(trajectory.back().path_pos_, trajectory_.back().path_vel_, false) >
            getVelocityMaxPathVelocityDeriv(trajectory_.back().path_pos_))
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
  assert(start1->path_pos_ < path_pos || tesseract_common::almostEqualRelativeAndAbs(start1->path_pos_, path_pos, EPS));

  while (start1 != start_trajectory.begin() || path_pos >= 0.0)
  {
    if (start1->path_pos_ < path_pos || tesseract_common::almostEqualRelativeAndAbs(start1->path_pos_, path_pos, EPS))
    {
      trajectory.emplace_front(path_pos, path_vel);
      path_vel -= time_step_ * acceleration;
      path_pos -= time_step_ * 0.5 * (path_vel + trajectory.front().path_vel_);
      acceleration = getMinMaxPathAcceleration(path_pos, path_vel, false);
      slope = (trajectory.front().path_vel_ - path_vel) / (trajectory.front().path_pos_ - path_pos);

      if (path_vel < 0.0)
      {
        valid_ = false;
        CONSOLE_BRIDGE_logError("Error while integrating backward: Negative path velocity");
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

    // It is possible to have both slope and start_slope to be equal
    // This occurs if two consecutive TrajectorySteps have the same acceleration.
    // Resulting in intersection_path_pos being nan because it divides by zero
    bool check_eq_slope = tesseract_common::almostEqualRelativeAndAbs(slope, start_slope, EPS);
    double intersection_path_pos{ 0 };
    if (check_eq_slope)
      intersection_path_pos = start1->path_pos_ + (start2->path_pos_ - start1->path_pos_) / 2.0;
    else
      intersection_path_pos =
          (start1->path_vel_ - path_vel + slope * path_pos - start_slope * start1->path_pos_) / (slope - start_slope);

    double pos_max = std::max(start1->path_pos_, path_pos);
    double pos_min = std::min(start2->path_pos_, trajectory.front().path_pos_);
    bool check1 = (pos_max < intersection_path_pos) ||
                  tesseract_common::almostEqualRelativeAndAbs(pos_max, intersection_path_pos, EPS);
    bool check2 = (intersection_path_pos < pos_min) ||
                  tesseract_common::almostEqualRelativeAndAbs(pos_min, intersection_path_pos, EPS);

    if (check1 && check2)
    {
      const double intersection_path_vel =
          start1->path_vel_ + (start_slope * (intersection_path_pos - start1->path_pos_));
      start_trajectory.erase(start2, start_trajectory.end());
      start_trajectory.emplace_back(intersection_path_pos, intersection_path_vel);
      start_trajectory.splice(start_trajectory.end(), trajectory);
      return;
    }
  }

  valid_ = false;
  CONSOLE_BRIDGE_logError("Error while integrating backward: Did not hit start trajectory");
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
    if (!tesseract_common::almostEqualRelativeAndAbs(config_deriv[i], 0.0, std::numeric_limits<double>::epsilon()))
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

bool Trajectory::assignData(InstructionsTrajectory& trajectory, const std::vector<std::size_t>& mapping) const
{
  const auto& dist_mapping = path_.getMapping();
  assert(trajectory.size() == static_cast<Eigen::Index>(mapping.size()));

  // Set Start
  PathData path_data = getPathData(0);
  Eigen::VectorXd uv = getVelocity(path_data).head(trajectory.dof());
  Eigen::VectorXd ua = getAcceleration(path_data).head(trajectory.dof());
  double time{ 0 };
  trajectory.setData(0, uv, ua, time);

  // Set intermidiate points
  double prev_time{ 0 };
  for (Eigen::Index i = 1; i < trajectory.size(); ++i)
  {
    std::size_t seg_idx = mapping[static_cast<std::size_t>(i)];

    time = getTime(dist_mapping.at(seg_idx));
    if (!(time > prev_time))
      time = prev_time + 1e-8;

    path_data = getPathData(time);
    uv = getVelocity(path_data).head(trajectory.dof());
    ua = getAcceleration(path_data).head(trajectory.dof());
    trajectory.setData(i, uv, ua, time);

    prev_time = time;
  }

  // Set end
  if (!(time > prev_time))
    time = prev_time + 1e-8;

  path_data = getPathData(time);
  uv = getVelocity(path_data).head(trajectory.dof());
  ua = getAcceleration(path_data).head(trajectory.dof());
  trajectory.setData((trajectory.size() - 1), uv, ua, time);

  assert(trajectory.isTimeStrictlyIncreasing());
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

  if (time < cached_time_)
  {
    cached_trajectory_segment_ = trajectory_.begin();
  }
  while (time >= cached_trajectory_segment_->time_)
  {
    ++cached_trajectory_segment_;
  }
  cached_time_ = time;
  return cached_trajectory_segment_;
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
    return trajectory_.begin();

  for (auto it = trajectory_.begin(); it != trajectory_.end(); ++it)
  {
    if (pos < it->path_pos_)
      return it;
  }

  throw std::runtime_error("Failed to find trajectory segment, this should not happen");
}

PathData Trajectory::getPathData(double time) const
{
  PathData data;

  auto it = getTrajectorySegment(time);
  auto previous = it;
  previous--;

  double time_step = it->time_ - previous->time_;
  const double acceleration =
      2.0 * (it->path_pos_ - previous->path_pos_ - time_step * previous->path_vel_) / (time_step * time_step);

  time_step = time - previous->time_;
  data.path_pos = (previous->path_pos_ + time_step * previous->path_vel_ + 0.5 * time_step * time_step * acceleration);
  data.path_vel = previous->path_vel_ + time_step * acceleration;
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

  double time_step = it->time_ - previous->time_;
  const double acceleration =
      2.0 * (it->path_pos_ - previous->path_pos_ - time_step * previous->path_vel_) / (time_step * time_step);

  const double a = 0.5 * acceleration;
  const double b = previous->path_vel_;
  const double c = previous->path_pos_ - pos;

  const double d = std::pow(b, 2.0) - (4 * a * c);
  const double e = ((d > 0) ? std::sqrt(d) : 0);
  const double dt = (-b + e) / (2.0 * a);
  assert(!(dt < 0));
  return (previous->time_ + dt);
}

Eigen::VectorXd Trajectory::getPosition(const PathData& data) const { return path_.getConfig(data.path_pos); }

Eigen::VectorXd Trajectory::getVelocity(const PathData& data) const
{
  return path_.getTangent(data.path_pos) * data.path_vel;
}

Eigen::VectorXd Trajectory::getAcceleration(const PathData& data) const
{
  Eigen::VectorXd path_acc =
      (path_.getTangent(data.path_pos) * data.path_vel - path_.getTangent(data.prev_path_pos) * data.prev_path_vel);
  double time_step = data.time - data.prev_time;
  if (time_step > 0.0)
    path_acc /= time_step;
  return path_acc;
}
}  // namespace totg
}  // namespace tesseract_planning
