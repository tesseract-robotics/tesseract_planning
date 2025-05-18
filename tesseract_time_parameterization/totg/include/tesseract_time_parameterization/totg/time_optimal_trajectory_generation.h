/*
 * Copyright (c) 2011-2012, Georgia Tech Research Corporation
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

#ifndef TESSERACT_TIME_PARAMETERIZATION_TIME_OPTIMAL_TRAJECTORY_GENERATION_H
#define TESSERACT_TIME_PARAMETERIZATION_TIME_OPTIMAL_TRAJECTORY_GENERATION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <list>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_time_parameterization/core/fwd.h>
#include <tesseract_time_parameterization/core/time_parameterization.h>

namespace tesseract_planning
{
class TimeOptimalTrajectoryGeneration : public TimeParameterization
{
public:
  TimeOptimalTrajectoryGeneration(std::string name);

  bool compute(CompositeInstruction& composite_instruction,
               const tesseract_environment::Environment& env,
               const tesseract_common::ProfileDictionary& profiles) const override;
};

namespace totg
{
class PathSegment
{
public:
  PathSegment() = default;
  PathSegment(double length) : length_(length) {}

  virtual ~PathSegment() = default;
  PathSegment(const PathSegment&) = default;
  PathSegment& operator=(const PathSegment&) = default;
  PathSegment(PathSegment&&) = default;
  PathSegment& operator=(PathSegment&&) = default;

  double getLength() const { return length_; }
  virtual Eigen::VectorXd getConfig(double s) const = 0;
  virtual Eigen::VectorXd getTangent(double s) const = 0;
  virtual Eigen::VectorXd getCurvature(double s) const = 0;
  virtual std::list<double> getSwitchingPoints() const = 0;
  virtual std::unique_ptr<PathSegment> clone() const = 0;

  double position_{ 0 };

protected:
  double length_{ 0 };
};

class Path
{
public:
  Path(const std::list<Eigen::VectorXd>& path, double max_deviation = 0.0);
  ~Path() = default;
  Path(const Path& path);
  Path& operator=(const Path&) = delete;
  Path(Path&&) = delete;
  Path& operator=(Path&&) = delete;

  double getLength() const;
  Eigen::VectorXd getConfig(double s) const;
  Eigen::VectorXd getTangent(double s) const;
  Eigen::VectorXd getCurvature(double s) const;
  double getNextSwitchingPoint(double s, bool& discontinuity) const;
  std::list<std::pair<double, bool>> getSwitchingPoints() const;
  const std::vector<double>& getMapping() const;

private:
  PathSegment* getPathSegment(double& s) const;
  double length_{ 0 };
  std::vector<double> mapping_;
  std::list<std::pair<double, bool>> switching_points_;
  std::list<std::unique_ptr<PathSegment>> path_segments_;
};

/** @brief Structure to store path data sampled at a point in time. */
struct PathData
{
  double prev_path_pos{ 0 };
  double prev_path_vel{ 0 };
  double prev_time{ 0 };
  double path_pos{ 0 };
  double path_vel{ 0 };
  double time{ 0 };
};

class Trajectory
{
public:
  /**
   * @brief Generates a time-optimal trajectory
   * @param path Path that will be parameterized
   * @param max_velocity Maximum velocity per joint
   * @param max_acceleration Maximum acceleration per joint
   * @param time_step Time step used when integrating forward and backward
   * @bug A negative path velocity error will occur if the path contains points that are close to but not equal to the
   * start or end point. A workaround (implemented in TimeOptimalTrajectoryGeneration::computeTimesteps) is to append an
   * extra joint that increments such that no point is ever repeated. See
   * https://github.com/ros-industrial-consortium/tesseract_planning/issues/27
   */
  Trajectory(const Path& path,
             const Eigen::VectorXd& max_velocity,
             const Eigen::VectorXd& max_acceleration,
             double time_step = 0.001);

  /** @brief Call this method after constructing the object to make sure the
     trajectory generation succeeded without errors. If this method returns
     false, all other methods have undefined behavior. **/
  bool isValid() const;

  /// @brief Returns the optimal duration of the trajectory
  double getDuration() const;

  /** @brief Return the path data for a given point in time */
  PathData getPathData(double time) const;
  /** @brief Return the position/configuration vector for a given point in time */
  Eigen::VectorXd getPosition(const PathData& data) const;
  /** @brief Return the velocity vector for a given point in time */
  Eigen::VectorXd getVelocity(const PathData& data) const;
  /** @brief Return the acceleration vector for a given point in time */
  Eigen::VectorXd getAcceleration(const PathData& data) const;
  /** @brief get the time given a position on the trajectory */
  double getTime(double pos) const;

  /**
   * @brief Assign trajectory velocity acceleration and time
   * @details This is brute force approach and should always return true
   */
  bool assignData(InstructionsTrajectory& trajectory, const std::vector<std::size_t>& mapping) const;

private:
  struct TrajectoryStep
  {
    TrajectoryStep() = default;
    TrajectoryStep(double path_pos, double path_vel) : path_pos_(path_pos), path_vel_(path_vel)
    {
      assert(!std::isnan(path_pos));
      assert(!std::isnan(path_vel));
    }
    double path_pos_{ 0 };
    double path_vel_{ 0 };
    double time_{ 0 };
  };

  bool getNextSwitchingPoint(double path_pos,
                             TrajectoryStep& next_switching_point,
                             double& before_acceleration,
                             double& after_acceleration);
  bool getNextAccelerationSwitchingPoint(double path_pos,
                                         TrajectoryStep& next_switching_point,
                                         double& before_acceleration,
                                         double& after_acceleration);
  bool getNextVelocitySwitchingPoint(double path_pos,
                                     TrajectoryStep& next_switching_point,
                                     double& before_acceleration,
                                     double& after_acceleration);
  bool integrateForward(std::list<TrajectoryStep>& trajectory, double acceleration);
  void
  integrateBackward(std::list<TrajectoryStep>& start_trajectory, double path_pos, double path_vel, double acceleration);
  double getMinMaxPathAcceleration(double path_position, double path_velocity, bool max);
  double getMinMaxPhaseSlope(double path_position, double path_velocity, bool max);
  double getAccelerationMaxPathVelocity(double path_pos) const;
  double getVelocityMaxPathVelocity(double path_pos) const;
  double getAccelerationMaxPathVelocityDeriv(double path_pos);
  double getVelocityMaxPathVelocityDeriv(double path_pos);

  std::list<TrajectoryStep>::const_iterator getTrajectorySegment(double time) const;
  std::list<TrajectoryStep>::const_iterator getTrajectorySegmentFromDist(double pos) const;

  Path path_;
  Eigen::VectorXd max_velocity_;
  Eigen::VectorXd max_acceleration_;
  Eigen::Index joint_num_;
  bool valid_{ true };
  std::list<TrajectoryStep> trajectory_;
  std::list<TrajectoryStep> end_trajectory_;  // non-empty only if the trajectory generation failed.

  const double time_step_;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)

  mutable double cached_time_;
  mutable std::list<TrajectoryStep>::const_iterator cached_trajectory_segment_;
};
}  // namespace totg
}  // namespace tesseract_planning

#endif
