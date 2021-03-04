/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Ken Anderson
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ken Anderson */

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <limits>
#include <cmath>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_time_parameterization/iterative_spline_parameterization.h>
#include <tesseract_common/utils.h>

namespace tesseract_planning
{

static void fit_cubic_spline(const long n, const double dt[], const double x[], double x1[], double x2[]);
static void adjust_two_positions(const long n,
                                 const double dt[],
                                 double x[],
                                 double x1[],
                                 double x2[],
                                 const double x2_i,
                                 const double x2_f);
static void init_times(long n, double dt[], const double x[], const double max_velocity[], const double min_velocity[]);
// static int fit_spline_and_adjust_times(const int n,
//                                       double dt[],
//                                       const double x[],
//                                       double x1[],
//                                       double x2[],
//                                       const double max_velocity,
//                                       const double min_velocity,
//                                       const double max_acceleration,
//                                       const double min_acceleration,
//                                       const double tfactor);
static double global_adjustment_factor(long n,
                                       double dt[],
                                       const double x[],
                                       double x1[],
                                       double x2[],
                                       const double max_velocity[],
                                       const double min_velocity[],
                                       const double max_acceleration[],
                                       const double min_acceleration[]);

// The path of a single joint: positions, velocities, and accelerations
struct SingleJointTrajectory
{
  std::vector<double> positions_;  // joint's position at time[x]
  std::vector<double> velocities_;
  std::vector<double> accelerations_;
  double initial_acceleration_{ 0 };
  double final_acceleration_{ 0 };
  std::vector<double> min_velocity_;
  std::vector<double> max_velocity_;
  std::vector<double> min_acceleration_;
  std::vector<double> max_acceleration_;
};

void globalAdjustment(std::vector<SingleJointTrajectory>& t2,
                      long num_joints,
                      long num_points,
                      std::vector<double>& time_diff);

IterativeSplineParameterization::IterativeSplineParameterization(bool add_points) : add_points_(add_points) {}

IterativeSplineParameterization::~IterativeSplineParameterization() = default;

bool IterativeSplineParameterization::compute(TrajectoryContainer& trajectory,
                                              const double& max_velocity,
                                              const double& max_acceleration,
                                              double max_velocity_scaling_factor,
                                              double max_acceleration_scaling_factor) const
{
  return compute(trajectory,
                 std::vector<double>(static_cast<std::size_t>(trajectory.dof()), max_velocity),
                 std::vector<double>(static_cast<std::size_t>(trajectory.dof()), max_acceleration),
                 max_velocity_scaling_factor,
                 max_acceleration_scaling_factor);
}

bool IterativeSplineParameterization::compute(TrajectoryContainer& trajectory,
                                              const std::vector<double>& max_velocity,
                                              const std::vector<double>& max_acceleration,
                                              double max_velocity_scaling_factor,
                                              double max_acceleration_scaling_factor) const
{
  return compute(trajectory,
                 Eigen::Map<const Eigen::VectorXd>(max_velocity.data(), static_cast<long>(max_velocity.size())),
                 Eigen::Map<const Eigen::VectorXd>(max_acceleration.data(), static_cast<long>(max_acceleration.size())),
                 max_velocity_scaling_factor,
                 max_acceleration_scaling_factor);
}

bool IterativeSplineParameterization::compute(TrajectoryContainer& trajectory,
                                              const Eigen::Ref<const Eigen::VectorXd>& max_velocity,
                                              const Eigen::Ref<const Eigen::VectorXd>& max_acceleration,
                                              double max_velocity_scaling_factor,
                                              double max_acceleration_scaling_factor) const
{
  Eigen::VectorXd max_velocity_scaling_factors =
      Eigen::VectorXd::Ones(static_cast<Eigen::Index>(trajectory.size())) * max_velocity_scaling_factor;
  Eigen::VectorXd max_acceleration_scaling_factors =
      Eigen::VectorXd::Ones(static_cast<Eigen::Index>(trajectory.size())) * max_acceleration_scaling_factor;
  return compute(
      trajectory, max_velocity, max_acceleration, max_velocity_scaling_factors, max_acceleration_scaling_factors);
}

bool IterativeSplineParameterization::compute(
    TrajectoryContainer& trajectory,
    const Eigen::Ref<const Eigen::VectorXd>& max_velocity,
    const Eigen::Ref<const Eigen::VectorXd>& max_acceleration,
    const Eigen::Ref<const Eigen::VectorXd>& max_velocity_scaling_factors,
    const Eigen::Ref<const Eigen::VectorXd>& max_acceleration_scaling_factors) const
{
  if (trajectory.empty())
    return true;

  Eigen::VectorXd velocity_scaling_factor = Eigen::VectorXd::Ones(trajectory.size());
  Eigen::VectorXd acceleration_scaling_factor = Eigen::VectorXd::Ones(trajectory.size());
  std::size_t num_points = static_cast<std::size_t>(trajectory.size());

  if (max_velocity.size() != trajectory.dof() || max_acceleration.size() != trajectory.dof())
    return false;

  // Set scaling factors
  if ((max_velocity_scaling_factors.array() > 0.0).all() && (max_velocity_scaling_factors.array() <= 1.0).all())
    velocity_scaling_factor = max_velocity_scaling_factors;
  else
  {
    for (Eigen::Index idx = 0; idx < max_velocity_scaling_factors.size(); idx++)
    {
      if (max_velocity_scaling_factors[idx] == 0.0)
        CONSOLE_BRIDGE_logDebug("iterative_spline_parameterization: A max_velocity_scaling_factor of 0.0 was "
                                "specified at index %d, "
                                "defaulting to %f instead.",
                                idx,
                                velocity_scaling_factor[idx]);
      else
        CONSOLE_BRIDGE_logWarn("iterative_spline_parameterization: Invalid max_velocity_scaling_factor %f specified at "
                               "index %d, "
                               "defaulting to %f instead.",
                               max_velocity_scaling_factors[idx],
                               idx,
                               velocity_scaling_factor[idx]);
    }
  }
  if ((max_acceleration_scaling_factors.array() > 0.0).all() && (max_acceleration_scaling_factors.array() <= 1.0).all())
    acceleration_scaling_factor = max_acceleration_scaling_factors;
  else
  {
    for (Eigen::Index idx = 0; idx < max_acceleration_scaling_factors.size(); idx++)
    {
      if (max_acceleration_scaling_factors[idx] == 0.0)
        CONSOLE_BRIDGE_logDebug("iterative_spline_parameterization: A max_acceleration_scaling_factor of 0.0 was "
                                "specified at index %d, defaulting to %f instead.",
                                idx,
                                acceleration_scaling_factor[idx]);
      else
        CONSOLE_BRIDGE_logWarn("iterative_spline_parameterization: Invalid max_acceleration_scaling_factor %f "
                               "specified "
                               "at index %d, "
                               "defaulting to %f instead.",
                               max_acceleration_scaling_factors[idx],
                               idx,
                               acceleration_scaling_factor[idx]);
    }
  }

  // JointTrajectory indexes in [point][joint] order.
  // We need [joint][point] order to solve efficiently,
  // so convert form here.

  std::vector<SingleJointTrajectory> t2(static_cast<std::size_t>(trajectory.dof()));

  const Eigen::VectorXd& start_vel = trajectory.getVelocity(0);
  const Eigen::VectorXd& last_vel = trajectory.getVelocity(static_cast<Eigen::Index>(num_points - 1));
  const Eigen::VectorXd& start_acc = trajectory.getAcceleration(0);
  const Eigen::VectorXd& last_acc = trajectory.getAcceleration(static_cast<Eigen::Index>(num_points - 1));

  for (std::size_t j = 0; j < static_cast<std::size_t>(trajectory.dof()); j++)
  {
    // Copy positions
    t2[j].positions_.resize(num_points, 0.0);
    for (std::size_t i = 0; i < num_points; i++)
      t2[j].positions_[i] = trajectory.getPosition(static_cast<Eigen::Index>(i))[static_cast<Eigen::Index>(j)];

    // Initialize velocities
    t2[j].velocities_.resize(num_points, 0.0);

    // Copy initial/final velocities if specified
    if (start_vel.size() > 0)
      t2[j].velocities_[0] = start_vel[static_cast<Eigen::Index>(j)];
    if (last_vel.size() > 0)
      t2[j].velocities_[num_points - 1] = last_vel[static_cast<Eigen::Index>(j)];

    // Initialize accelerations
    t2[j].accelerations_.resize(num_points, 0.0);
    t2[j].initial_acceleration_ = 0.0;
    t2[j].final_acceleration_ = 0.0;

    // Copy initial/final accelerations if specified
    if (start_acc.size() > 0)
      t2[j].initial_acceleration_ = start_acc[static_cast<Eigen::Index>(j)];
    t2[j].accelerations_[0] = t2[j].initial_acceleration_;
    if (last_acc.size() > 0)
      t2[j].final_acceleration_ = last_acc[static_cast<Eigen::Index>(j)];
    t2[j].accelerations_[num_points - 1] = t2[j].final_acceleration_;

    // Set bounds based on inputs
    t2[j].max_velocity_.resize(num_points);
    t2[j].min_velocity_.resize(num_points);
    t2[j].max_acceleration_.resize(num_points);
    t2[j].min_acceleration_.resize(num_points);
    Eigen::Map<Eigen::VectorXd> max_velocity_eigen(t2[j].max_velocity_.data(), static_cast<Eigen::Index>(num_points));
    Eigen::Map<Eigen::VectorXd> min_velocity_eigen(t2[j].min_velocity_.data(), static_cast<Eigen::Index>(num_points));
    Eigen::Map<Eigen::VectorXd> max_acceleration_eigen(t2[j].max_acceleration_.data(),
                                                       static_cast<Eigen::Index>(num_points));
    Eigen::Map<Eigen::VectorXd> min_acceleration_eigen(t2[j].min_acceleration_.data(),
                                                       static_cast<Eigen::Index>(num_points));

    max_velocity_eigen =
        Eigen::VectorXd::Ones(static_cast<Eigen::Index>(num_points)) * max_velocity[static_cast<Eigen::Index>(j)];
    min_velocity_eigen =
        Eigen::VectorXd::Ones(static_cast<Eigen::Index>(num_points)) * -max_velocity[static_cast<Eigen::Index>(j)];
    max_velocity_eigen.array() *= velocity_scaling_factor.array();
    min_velocity_eigen.array() *= velocity_scaling_factor.array();

    max_acceleration_eigen =
        Eigen::VectorXd::Ones(static_cast<Eigen::Index>(num_points)) * max_acceleration[static_cast<Eigen::Index>(j)];
    min_acceleration_eigen =
        Eigen::VectorXd::Ones(static_cast<Eigen::Index>(num_points)) * -max_acceleration[static_cast<Eigen::Index>(j)];
    max_acceleration_eigen.array() *= acceleration_scaling_factor.array();
    min_acceleration_eigen.array() *= acceleration_scaling_factor.array();

    // Error out if bounds don't make sense
    if ((max_velocity_eigen.array() <= 0.0).any() || (max_acceleration_eigen.array() <= 0.0).any())
    {
      CONSOLE_BRIDGE_logError("iterative_spline_parameterization: Joint %d max velocity %f and max acceleration %f "
                              "must be greater than zero or a solution won't be found.",
                              j,
                              t2[j].max_velocity_[0],
                              t2[j].max_acceleration_[0]);
      return false;
    }
    if ((min_velocity_eigen.array() >= 0.0).any() || (min_acceleration_eigen.array() >= 0.0).any())
    {
      CONSOLE_BRIDGE_logError("trajectory_processing.iterative_spline_parameterization: Joint %d min velocity %f and "
                              "min acceleration %f must be less than zero or a solution won't be found.",
                              j,
                              t2[j].min_velocity_[0],
                              t2[j].min_acceleration_[0]);
      return false;
    }
  }

  bool add_points = add_points_;
  if (num_points < 2)
    add_points = false;

  if (add_points)
  {
    // Insert 2nd and 2nd-last points
    // (required to force acceleration to specified values at endpoints)
    for (unsigned int j = 0; j < trajectory.dof(); j++)
    {
      double value = 0.9 * t2[j].positions_[0] + 0.1 * t2[j].positions_[1];
      t2[j].positions_.insert(t2[j].positions_.begin() + 1, value);

      value = t2[j].velocities_.front();
      t2[j].velocities_.insert(t2[j].velocities_.begin() + 1, value);

      value = t2[j].accelerations_.front();
      t2[j].accelerations_.insert(t2[j].accelerations_.begin() + 1, value);

      value = t2[j].max_velocity_.front();
      t2[j].max_velocity_.insert(t2[j].max_velocity_.begin() + 1, value);

      value = t2[j].min_velocity_.front();
      t2[j].min_velocity_.insert(t2[j].min_velocity_.begin() + 1, value);

      value = t2[j].max_acceleration_.front();
      t2[j].max_acceleration_.insert(t2[j].max_acceleration_.begin() + 1, value);

      value = t2[j].min_acceleration_.front();
      t2[j].min_acceleration_.insert(t2[j].min_acceleration_.begin() + 1, value);
    }
    num_points++;

    for (unsigned int j = 0; j < trajectory.dof(); j++)
    {
      double value = 0.1 * t2[j].positions_[num_points - 2] + 0.9 * t2[j].positions_[num_points - 1];
      t2[j].positions_.insert(t2[j].positions_.end() - 1, value);

      value = t2[j].velocities_.back();
      t2[j].velocities_.insert(t2[j].velocities_.end() - 1, value);

      value = t2[j].accelerations_.back();
      t2[j].accelerations_.insert(t2[j].accelerations_.end() - 1, value);

      value = t2[j].max_velocity_.back();
      t2[j].max_velocity_.insert(t2[j].max_velocity_.end() - 1, value);

      value = t2[j].min_velocity_.back();
      t2[j].min_velocity_.insert(t2[j].min_velocity_.end() - 1, value);

      value = t2[j].max_acceleration_.back();
      t2[j].max_acceleration_.insert(t2[j].max_acceleration_.end() - 1, value);

      value = t2[j].min_acceleration_.back();
      t2[j].min_acceleration_.insert(t2[j].min_acceleration_.end() - 1, value);
    }
    num_points++;
  }

  // Error check
  if (num_points < 4)
  {
    CONSOLE_BRIDGE_logError("iterative_spline_parameterization: number of waypoints %d, needs to be greater than 3.",
                            num_points);
    return false;
  }
  for (std::size_t j = 0; j < static_cast<std::size_t>(trajectory.dof()); j++)
  {
    if (t2[j].velocities_[0] > t2[j].max_velocity_[0] || t2[j].velocities_[0] < t2[j].min_velocity_[0])
    {
      CONSOLE_BRIDGE_logError("iterative_spline_parameterization: Initial velocity %f out of bounds.",
                              t2[j].velocities_[0]);
      return false;
    }
    else if (t2[j].velocities_[num_points - 1] > t2[j].max_velocity_[num_points - 1] ||
             t2[j].velocities_[num_points - 1] < t2[j].min_velocity_[num_points - 1])
    {
      CONSOLE_BRIDGE_logError("iterative_spline_parameterization: Final velocity %f out of bounds.",
                              t2[j].velocities_[num_points - 1]);
      return false;
    }
    else if (t2[j].accelerations_[0] > t2[j].max_acceleration_[0] ||
             t2[j].accelerations_[0] < t2[j].min_acceleration_[0])
    {
      CONSOLE_BRIDGE_logError("iterative_spline_parameterization: Initial acceleration %f out of bounds\n",
                              t2[j].accelerations_[0]);
      return false;
    }
    else if (t2[j].accelerations_[num_points - 1] > t2[j].max_acceleration_[num_points - 1] ||
             t2[j].accelerations_[num_points - 1] < t2[j].min_acceleration_[num_points - 1])
    {
      CONSOLE_BRIDGE_logError("iterative_spline_parameterization: Final acceleration %f out of bounds\n",
                              t2[j].accelerations_[num_points - 1]);
      return false;
    }
  }

  // Initialize times
  // start with valid velocities, then expand intervals
  // epsilon to prevent divide-by-zero
  std::vector<double> time_diff(static_cast<std::size_t>(num_points - 1), std::numeric_limits<double>::epsilon());
  for (unsigned int j = 0; j < trajectory.dof(); j++)
    init_times(static_cast<long>(num_points),
               &time_diff[0],
               &t2[j].positions_[0],
               &t2[j].max_velocity_[0],
               &t2[j].min_velocity_[0]);

  // Stretch intervals until close to the bounds
  while (1)
  {
    int loop = 0;

    // Calculate the interval stretches due to acceleration
    std::vector<double> time_factor(static_cast<std::size_t>(num_points - 1), 1.00);
    for (unsigned int j = 0; j < trajectory.dof(); j++)
    {
      // Move points to satisfy initial/final acceleration
      if (add_points)
      {
        adjust_two_positions(static_cast<int>(num_points),
                             &time_diff[0],
                             &t2[j].positions_[0],
                             &t2[j].velocities_[0],
                             &t2[j].accelerations_[0],
                             t2[j].initial_acceleration_,
                             t2[j].final_acceleration_);
      }

      fit_cubic_spline(static_cast<int>(num_points),
                       &time_diff[0],
                       &t2[j].positions_[0],
                       &t2[j].velocities_[0],
                       &t2[j].accelerations_[0]);
      for (unsigned i = 0; i < num_points; i++)
      {
        const double acc = t2[j].accelerations_[i];
        double atfactor = 1.0;
        if (acc > t2[j].max_acceleration_[i])
          atfactor = sqrt(acc / t2[j].max_acceleration_[i]);
        if (acc < t2[j].min_acceleration_[i])
          atfactor = sqrt(acc / t2[j].min_acceleration_[i]);
        if (atfactor > 1.01)  // within 1%
          loop = 1;
        atfactor = (atfactor - 1.0) / 16.0 + 1.0;  // 1/16th
        if (i > 0)
          time_factor[i - 1] = std::max(time_factor[i - 1], atfactor);
        if (i < num_points - 1)
          time_factor[i] = std::max(time_factor[i], atfactor);
      }
    }

    if (loop == 0)
      break;  // finished

    // Stretch
    for (unsigned i = 0; i < num_points - 1; i++)
      time_diff[i] *= time_factor[i];
  }

  // Final adjustment forces the trajectory within bounds
  globalAdjustment(t2, trajectory.dof(), static_cast<long>(num_points), time_diff);

  // Convert back to JointTrajectory form
  double time = 0;
  Eigen::Index idx = 0;
  for (unsigned int i = 0; i < num_points; i++)
  {
    Eigen::VectorXd uv(trajectory.dof());
    Eigen::VectorXd ua(trajectory.dof());
    for (unsigned int j = 0; j < trajectory.dof(); j++)
    {
      uv[j] = t2[j].velocities_[i];
      ua[j] = t2[j].accelerations_[i];
    }

    // Calculate time from start
    if (i > 0)
      time = time + time_diff[i - 1];

    // Do not process added points
    if (add_points && (i == 1 || i == num_points - 2))
    {
      time = time + time_diff[i - 1];
      continue;
    }

    trajectory.setData(idx++, uv, ua, time);
  }

  return true;
}

//////// Internal functions //////////////

/*
  Fit a 'clamped' cubic spline over a series of points.
  A cubic spline ensures continuous function across positions,
  1st derivative (velocities), and 2nd derivative (accelerations).
  'Clamped' means the first derivative at the endpoints is specified.

  Fitting a cubic spline involves solving a series of linear equations.
  The general form for each segment is:
    (tj-t_(j-1))*x"_(j-1) + 2*(t_(j+1)-t_(j-1))*x"j + (t_(j+1)-tj)*x"_j+1) =
          (x_(j+1)-xj)/(t_(j+1)-tj) - (xj-x_(j-1))/(tj-t_(j-1))
  And the first and last segment equations are clamped to specified values: x1_i and x1_f.

  Represented in matrix form:
  [ 2*(t1-t0)   (t1-t0)                              0              ][x0"]       [(x1-x0)/(t1-t0) - t1_i           ]
  [ t1-t0       2*(t2-t0)   t2-t1                                   ][x1"]       [(x2-x1)/(t2-t1) - (x1-x0)/(t1-t0)]
  [             t2-t1       2*(t3-t1)   t3-t2                       ][x2"] = 6 * [(x3-x2)/(t3/t2) - (x2-x1)/(t2-t1)]
  [                       ...         ...         ...               ][...]       [...                              ]
  [ 0                                    tN-t_(N-1)  2*(tN-t_(N-1)) ][xN"]       [t1_f - (xN-x_(N-1))/(tN-t_(N-1)) ]

  This matrix is tridiagonal, which can be solved solved in O(N) time
  using the tridiagonal algorithm.
  There is a forward propogation pass followed by a backsubstitution pass.

  n is the number of points
  dt contains the time difference between each point (size=n-1)
  x  contains the positions                          (size=n)
  x1 contains the 1st derivative (velocities)        (size=n)
     x1[0] and x1[n-1] MUST be specified.
  x2 contains the 2nd derivative (accelerations)     (size=n)
  x1 and x2 are filled in by the algorithm.
*/

static void fit_cubic_spline(const long n, const double dt[], const double x[], double x1[], double x2[])
{
  long i;
  const double x1_i = x1[0], x1_f = x1[n - 1];

  // Tridiagonal alg - forward sweep
  // x1 and x2 used to store the temporary coefficients c and d
  // (will get overwritten during backsubstitution)
  double *c = x1, *d = x2;
  c[0] = 0.5;
  d[0] = 3.0 * ((x[1] - x[0]) / dt[0] - x1_i) / dt[0];
  for (i = 1; i <= n - 2; i++)
  {
    const double dt2 = dt[i - 1] + dt[i];
    const double a = dt[i - 1] / dt2;
    const double denom = 2.0 - a * c[i - 1];
    c[i] = (1.0 - a) / denom;
    d[i] = 6.0 * ((x[i + 1] - x[i]) / dt[i] - (x[i] - x[i - 1]) / dt[i - 1]) / dt2;
    d[i] = (d[i] - a * d[i - 1]) / denom;
  }
  const double denom = dt[n - 2] * (2.0 - c[n - 2]);
  d[n - 1] = 6.0 * (x1_f - (x[n - 1] - x[n - 2]) / dt[n - 2]);
  d[n - 1] = (d[n - 1] - dt[n - 2] * d[n - 2]) / denom;

  // Tridiagonal alg - backsubstitution sweep
  // 2nd derivative
  x2[n - 1] = d[n - 1];
  for (i = n - 2; i >= 0; i--)
    x2[i] = d[i] - c[i] * x2[i + 1];

  // 1st derivative
  x1[0] = x1_i;
  for (i = 1; i < n - 1; i++)
    x1[i] = (x[i + 1] - x[i]) / dt[i] - (2 * x2[i] + x2[i + 1]) * dt[i] / 6.0;
  x1[n - 1] = x1_f;
}

/*
  Modify the value of x[1] and x[N-2]
  so that 2nd derivative starts and ends at specified value.
  This involves fitting the spline twice,
  then solving for the specified value.

  x2_i and x2_f are the (initial and final) 2nd derivative at 0 and N-1
*/

static void adjust_two_positions(const long n,
                                 const double dt[],
                                 double x[],
                                 double x1[],
                                 double x2[],
                                 const double x2_i,
                                 const double x2_f)
{
  x[1] = x[0];
  x[n - 2] = x[n - 3];
  fit_cubic_spline(n, dt, x, x1, x2);
  double a0 = x2[0];
  double b0 = x2[n - 1];

  x[1] = x[2];
  x[n - 2] = x[n - 1];
  fit_cubic_spline(n, dt, x, x1, x2);
  double a2 = x2[0];
  double b2 = x2[n - 1];

  // we can solve this with linear equation (use two-point form)
  // if (a2 != a0)
  if (!tesseract_common::almostEqualRelativeAndAbs(a2, a0, 1e-5))
    x[1] = x[0] + ((x[2] - x[0]) / (a2 - a0)) * (x2_i - a0);

  // if (b2 != b0)
  if (!tesseract_common::almostEqualRelativeAndAbs(b2, b0, 1e-5))
    x[n - 2] = x[n - 3] + ((x[n - 1] - x[n - 3]) / (b2 - b0)) * (x2_f - b0);
}

/*
  Find time required to go max velocity on each segment.
  Increase a segment's time interval if the current time isn't long enough.
*/

static void init_times(long n, double dt[], const double x[], const double max_velocity[], const double min_velocity[])
{
  for (long i = 0; i < n - 1; i++)
  {
    double time;
    double dx = x[i + 1] - x[i];
    if (dx >= 0.0)
      time = (dx / max_velocity[i]);
    else
      time = (dx / min_velocity[i]);
    time += std::numeric_limits<double>::epsilon();  // prevent divide-by-zero

    if (dt[i] < time)
      dt[i] = time;
  }
}

/*
  Fit a spline, then check each interval to see if bounds are met.
  If all bounds met (no time adjustments made), return 0.
  If bounds not met (time adjustments made), slightly increase the
  surrounding time intervals and return 1.

  n is the number of points
  dt contains the time difference between each point (size=n-1)
  x  contains the positions                          (size=n)
  x1 contains the 1st derivative (velocities)        (size=n)
     x1[0] and x1[n-1] MUST be specified.
  x2 contains the 2nd derivative (accelerations)     (size=n)
  max_velocity is the max velocity for this joint.
  min_velocity is the min velocity for this joint.
  max_acceleration is the max acceleration for this joint.
  min_acceleration is the min acceleration for this joint.
  tfactor is the time adjustment (multiplication) factor.
  x1 and x2 are filled in by the algorithm.
*/

// static int fit_spline_and_adjust_times(const int n,
//                                       double dt[],
//                                       const double x[],
//                                       double x1[],
//                                       double x2[],
//                                       const double max_velocity,
//                                       const double min_velocity,
//                                       const double max_acceleration,
//                                       const double min_acceleration,
//                                       const double tfactor)
//{
//  int i, ret = 0;

//  fit_cubic_spline(n, dt, x, x1, x2);

//  // Instantaneous velocity is calculated at each point
//  for (i = 0; i < n - 1; i++)
//  {
//    const double vel = x1[i];
//    const double vel2 = x1[i + 1];
//    if (vel > max_velocity || vel < min_velocity || vel2 > max_velocity || vel2 < min_velocity)
//    {
//      dt[i] *= tfactor;
//      ret = 1;
//    }
//  }
//  // Instantaneous acceleration is calculated at each point
//  if (ret == 0)
//  {
//    for (i = 0; i < n - 1; i++)
//    {
//      const double acc = x2[i];
//      const double acc2 = x2[i + 1];
//      if (acc > max_acceleration || acc < min_acceleration || acc2 > max_acceleration || acc2 < min_acceleration)
//      {
//        dt[i] *= tfactor;
//        ret = 1;
//      }
//    }
//  }

//  return ret;
//}

// return global expansion multiplicative factor required
// to force within bounds.
// Assumes that the spline is already fit
// (fit_cubic_spline must have been called before this).
static double global_adjustment_factor(long n,
                                       double /*dt*/[],
                                       const double /*x*/[],
                                       double x1[],
                                       double x2[],
                                       const double max_velocity[],
                                       const double min_velocity[],
                                       const double max_acceleration[],
                                       const double min_acceleration[])
{
  double tfactor2 = 1.00;

  // fit_cubic_spline(n, dt, x, x1, x2);

  for (long i = 0; i < n; i++)
  {
    double tfactor;
    tfactor = x1[i] / max_velocity[i];
    if (tfactor2 < tfactor)
      tfactor2 = tfactor;
    tfactor = x1[i] / min_velocity[i];
    if (tfactor2 < tfactor)
      tfactor2 = tfactor;

    if (x2[i] >= 0)
    {
      tfactor = sqrt(fabs(x2[i] / max_acceleration[i]));
      if (tfactor2 < tfactor)
        tfactor2 = tfactor;
    }
    else
    {
      tfactor = sqrt(fabs(x2[i] / min_acceleration[i]));
      if (tfactor2 < tfactor)
        tfactor2 = tfactor;
    }
  }

  return tfactor2;
}

// Expands the entire trajectory to fit exactly within bounds
void globalAdjustment(std::vector<SingleJointTrajectory>& t2,
                      long num_joints,
                      long num_points,
                      std::vector<double>& time_diff)
{
  double gtfactor = 1.0;
  for (std::size_t j = 0; j < static_cast<std::size_t>(num_joints); j++)
  {
    double tfactor;
    tfactor = global_adjustment_factor(num_points,
                                       &time_diff[0],
                                       &t2[j].positions_[0],
                                       &t2[j].velocities_[0],
                                       &t2[j].accelerations_[0],
                                       &t2[j].max_velocity_[0],
                                       &t2[j].min_velocity_[0],
                                       &t2[j].max_acceleration_[0],
                                       &t2[j].min_acceleration_[0]);
    if (tfactor > gtfactor)
      gtfactor = tfactor;
  }

  // printf("# Global adjustment: %0.4f%%\n", 100.0 * (gtfactor - 1.0));
  for (std::size_t i = 0; i < static_cast<std::size_t>(num_points - 1); i++)
    time_diff[i] *= gtfactor;

  for (std::size_t j = 0; j < static_cast<std::size_t>(num_joints); j++)
  {
    fit_cubic_spline(num_points, &time_diff[0], &t2[j].positions_[0], &t2[j].velocities_[0], &t2[j].accelerations_[0]);
  }
}
}  // namespace tesseract_planning
