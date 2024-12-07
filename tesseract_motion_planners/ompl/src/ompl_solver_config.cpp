/**
 * @file ompl_solver_config.cpp
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

#include <tesseract_motion_planners/ompl/ompl_solver_config.h>
#include <tesseract_motion_planners/ompl/ompl_planner_configurator.h>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/nvp.hpp>

namespace tesseract_planning
{
template <class Archive>
void OMPLSolverConfig::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(planning_time);
  ar& BOOST_SERIALIZATION_NVP(max_solutions);
  ar& BOOST_SERIALIZATION_NVP(simplify);
  ar& BOOST_SERIALIZATION_NVP(optimize);
  ar& BOOST_SERIALIZATION_NVP(planners);
}
}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::OMPLSolverConfig)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::OMPLSolverConfig)
