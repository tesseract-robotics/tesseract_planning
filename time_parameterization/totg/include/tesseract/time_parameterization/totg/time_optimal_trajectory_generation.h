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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <limits>
#include <list>
#include <memory>
#include <cassert>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/time_parameterization/fwd.h>
#include <tesseract/time_parameterization/time_parameterization.h>

namespace tesseract::time_parameterization
{
class TimeOptimalTrajectoryGeneration : public TimeParameterization
{
public:
  TimeOptimalTrajectoryGeneration(std::string name);

  /**
   * @brief Time the program's move instructions
   * @details A move instruction within the profile's min_angle_change of the last one kept, in every joint, is removed,
   * so the program can come back with fewer of them, and a composite made only of such repeats with none. At the end of
   * the program the last move of such a run, the goal, is kept instead of the first, unless another instruction lies
   * between them; the program then ends at the run's first move, within min_angle_change of the goal. The goal also
   * replaces the kept moves before it that are within min_angle_change of it, so a removed move lies within twice
   * min_angle_change of a kept one. When nothing moves, only the first move instruction is kept. Other instructions
   * keep their place relative to the motion. Where the path turns back, or a corner would blend at a radius below
   * 1e-4, the motion stops: the waypoint there has zero velocity. Throws if the profile's min_angle_change is below
   * 1e-5. The motion starts and ends at rest, unless the profile's use_start_velocity or use_end_velocity takes the
   * first or last waypoint's velocity instead.
   *
   * A waypoint at a blended corner is moved onto the blend, where its time, velocity and acceleration are taken, by up
   * to the profile's path_tolerance (the Euclidean distance in joint space). It stays within the joint ranges of the
   * waypoints around it, but contact checks run before this do not see the moved position, and callers that match
   * output waypoints to input ones by position do not find it; an instruction such as I/O that follows it runs there.
   * The motion is exact at the waypoints only: between them
   * a controller's interpolation does not follow the time-optimal profile, so densify the input where that matters.
   */
  bool compute(tesseract::command_language::CompositeInstruction& composite_instruction,
               const tesseract::environment::Environment& env,
               const tesseract::common::ProfileDictionary& profiles) const override;
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

/**
 * @brief A piecewise linear path through waypoints, with its corners blended
 * @details getConfig, getTangent and getCurvature resume their segment lookup from the last one, so they must not be
 * called concurrently on one object.
 */
class Path
{
public:
  /**
   * @brief Build a path through the waypoints, blending each corner within max_deviation
   * @details With max_deviation <= 0, the default included, the corners are left unblended, and a Trajectory along
   * them reports velocity jumps that no finite acceleration can follow. TimeOptimalTrajectoryGeneration::compute
   * rejects such a tolerance.
   * @throws std::invalid_argument with fewer than two waypoints
   */
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
  /**
   * @brief Whether the path turns back on itself at a waypoint, where the path velocity must come to zero; a Trajectory
   * along it is invalid, so split such a path there and time the pieces from rest to rest
   */
  bool hasReversal() const;
  const std::list<std::pair<double, bool>>& getSwitchingPoints() const&;
  /** @brief A copy, so that it outlives the temporary path */
  std::list<std::pair<double, bool>> getSwitchingPoints() const&&;
  const std::vector<double>& getMapping() const;
  /**
   * @brief Whether waypoint i is a corner blended by an arc of positive length; the path then misses it, and passes the
   * arc's middle at getMapping()[i]
   */
  bool isBlended(std::size_t i) const;

private:
  PathSegment* getPathSegment(double& s) const;
  double length_{ 0 };
  std::vector<double> mapping_;
  std::vector<bool> blended_;
  std::list<std::pair<double, bool>> switching_points_;
  bool has_reversal_{ false };
  std::list<std::unique_ptr<PathSegment>> path_segments_;
  /** @brief The segment the last lookup found, where the next one starts */
  mutable std::list<std::unique_ptr<PathSegment>>::const_iterator cursor_;
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
  /** @brief The path acceleration, constant over the integration step that contains time */
  double path_acc{ 0 };
};

/**
 * @brief A time-optimal trajectory along a path
 * @details getPathData and getTime resume their step lookup from the last one, and getPosition, getVelocity and
 * getAcceleration look up the path, so none of them may be called concurrently on one object.
 */
class Trajectory
{
public:
  /**
   * @brief Generates a time-optimal trajectory
   * @param path Path that will be parameterized
   * @param max_velocity Maximum velocity per joint
   * @param max_acceleration Maximum acceleration per joint
   * @param time_step Time step used when integrating forward and backward
   * @details Invalid if the time step is not positive or the path has a reversal (Path::hasReversal)
   */
  Trajectory(const Path& path,
             const Eigen::VectorXd& max_velocity,
             const Eigen::VectorXd& max_acceleration,
             double time_step = 0.001);

  /**
   * @brief Generates a time-optimal trajectory that starts moving along the path at start_path_velocity and ends at
   * end_path_velocity
   * @details Also invalid if either velocity is negative or not finite, if it exceeds the limit at its end of the path
   * (within rounding of it, the limit is taken instead), if the path does not leave room to slow down from the start
   * velocity for a later limit, or if the end velocity is out of reach
   */
  Trajectory(const Path& path,
             const Eigen::VectorXd& max_velocity,
             const Eigen::VectorXd& max_acceleration,
             double time_step,
             double start_path_velocity,
             double end_path_velocity = 0.0);

  /** @brief Call this method after constructing the object to make sure the
     trajectory generation succeeded without errors. If this method returns
     false, all other methods have undefined behavior. **/
  bool isValid() const;

  /// @brief Returns the optimal duration of the trajectory
  double getDuration() const;

  /**
   * @brief Return the path data for a given point in time
   * @details A time before 0 or after getDuration() extrapolates the first or last step, and a NaN time gives a NaN
   * path position and velocity
   */
  PathData getPathData(double time) const;
  /** @brief Return the position/configuration vector for a given point in time */
  Eigen::VectorXd getPosition(const PathData& data) const;
  /** @brief Return the velocity vector for a given point in time */
  Eigen::VectorXd getVelocity(const PathData& data) const;
  /**
   * @brief Return the acceleration vector for a given point in time
   * @details Reads path_pos, path_vel and path_acc only, as getPathData sets them
   */
  Eigen::VectorXd getAcceleration(const PathData& data) const;
  /**
   * @brief get the time given a position on the trajectory
   * @details pos must lie within [0, the path's length]. Outside it, debug builds assert, and other builds read the
   * time off the first or last step
   */
  double getTime(double pos) const;

  /**
   * @brief Write time, velocity and acceleration to the waypoints at the path's points, and move waypoints at blended
   * corners onto the blend, where the path passes them
   * @param trajectory The waypoints
   * @param kept For each point of the path, the index in trajectory of the waypoint at it; waypoints not listed are
   * left as they are
   * @return False, leaving the waypoints unchanged, if kept does not match the path or the times would not strictly
   * increase
   */
  bool assignData(InstructionsTrajectory& trajectory, const std::vector<std::size_t>& kept) const;

  /** @brief As assignData, with every time written time_offset later, for a trajectory that continues another */
  bool assignData(InstructionsTrajectory& trajectory, const std::vector<std::size_t>& kept, double time_offset) const;

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
    /** @brief The path acceleration of the step that ends here */
    double path_acc_{ 0 };
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

  /** @brief The last time and path position looked up, and the steps they found; a copy starts over */
  struct LookupCache
  {
    LookupCache() = default;
    ~LookupCache() = default;
    LookupCache(const LookupCache& /*other*/) {}
    LookupCache& operator=(const LookupCache&) = delete;
    LookupCache(LookupCache&&) = delete;
    LookupCache& operator=(LookupCache&&) = delete;

    /** @brief NaN until the first lookup, so that one starts at the first step */
    double time{ std::numeric_limits<double>::quiet_NaN() };
    std::list<TrajectoryStep>::const_iterator time_segment;
    double path_pos{ std::numeric_limits<double>::quiet_NaN() };
    std::list<TrajectoryStep>::const_iterator path_pos_segment;
  };
  mutable LookupCache lookup_;
};
}  // namespace totg
}  // namespace tesseract::time_parameterization

#endif
