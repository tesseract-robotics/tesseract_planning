/**
 * @file ompl_solver_config.h
 * @brief Tesseract OMPL solver config
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
#ifndef TESSERACT_MOTION_PLANNERS_OMPL_OMPL_SOLVER_CONFIG_H
#define TESSERACT_MOTION_PLANNERS_OMPL_OMPL_SOLVER_CONFIG_H

#include <memory>
#include <vector>
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>

namespace tesseract_planning
{
struct OMPLPlannerConfigurator;
struct OMPLSolverConfig
{
  using Ptr = std::shared_ptr<OMPLSolverConfig>;
  using ConstPtr = std::shared_ptr<const OMPLSolverConfig>;
  using UPtr = std::unique_ptr<OMPLSolverConfig>;
  using ConstUPtr = std::unique_ptr<const OMPLSolverConfig>;

  virtual ~OMPLSolverConfig() = default;

  /** @brief Max planning time allowed in seconds */
  double planning_time = 5.0;

  /** @brief The max number of solutions. If max solutions are hit it will exit even if other threads are running. */
  int max_solutions = 10;

  /**
   * @brief Simplify trajectory.
   *
   * Note: If set to true it ignores n_output_states and returns the simplest trajectory.
   */
  bool simplify = false;

  /**
   * @brief This uses all available planning time to create the most optimized trajectory given the objective function.
   *
   * This is required because not all OMPL planners are optimize graph planners. If the planner you choose is an
   * optimize graph planner then setting this to true has no affect. In the case of non-optimize planners they still
   * use the OptimizeObjective function but only when searching the graph to find the most optimize solution based
   * on the provided optimize objective function. In the case of these type of planners like RRT and RRTConnect if set
   * to true it will leverage all planning time to keep finding solutions up to your max solutions count to find the
   * most optimal solution.
   */
  bool optimize = true;

  /**
   * @brief The planner configurators
   *
   * This defaults to two RRTConnectConfigurator
   *
   * This will create a new thread for each planner configurator provided.
   */
  std::vector<std::shared_ptr<const OMPLPlannerConfigurator>> planners;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::OMPLSolverConfig)

#endif  // TESSERACT_MOTION_PLANNERS_OMPL_OMPL_SOLVER_CONFIG_H
