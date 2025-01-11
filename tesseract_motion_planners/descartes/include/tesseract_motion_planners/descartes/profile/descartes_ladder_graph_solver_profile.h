/**
 * @file descartes_ladder_graph_solver_profile.h
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
#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_DESCARTES_LADDER_GRAPH_SOLVER_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_DESCARTES_DESCARTES_LADDER_GRAPH_SOLVER_PROFILE_H

#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>

namespace tesseract_planning
{
template <typename FloatType>
class DescartesLadderGraphSolverProfile : public DescartesSolverProfile<FloatType>
{
public:
  using Ptr = std::shared_ptr<DescartesLadderGraphSolverProfile<FloatType>>;
  using ConstPtr = std::shared_ptr<const DescartesLadderGraphSolverProfile<FloatType>>;

  DescartesLadderGraphSolverProfile() = default;

  /** @brief Number of threads to use during planning */
  int num_threads{ 1 };

  std::unique_ptr<descartes_light::Solver<FloatType>> create() const override;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

using DescartesLadderGraphSolverProfileF = DescartesLadderGraphSolverProfile<float>;
using DescartesLadderGraphSolverProfileD = DescartesLadderGraphSolverProfile<double>;
}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::DescartesLadderGraphSolverProfile<float>)
BOOST_CLASS_EXPORT_KEY(tesseract_planning::DescartesLadderGraphSolverProfile<double>)

#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_DESCARTES_LADDER_GRAPH_SOLVER_PROFILE_H
