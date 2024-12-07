/**
 * @file ompl_planner_configurator.cpp
 * @brief Tesseract OMPL planner configurators
 *
 * @author Levi Armstrong
 * @date February 1, 2020
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/Planner.h>

#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>

#include <boost/serialization/nvp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/ompl_planner_configurator.h>
#include <tesseract_common/utils.h>

namespace tesseract_planning
{
template <class Archive>
void OMPLPlannerConfigurator::serialize(Archive& /*ar*/, const unsigned int /*version*/)
{
}

ompl::base::PlannerPtr SBLConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::SBL>(si);
  planner->setRange(range);
  return planner;
}

OMPLPlannerType SBLConfigurator::getType() const { return OMPLPlannerType::SBL; }

template <class Archive>
void SBLConfigurator::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(OMPLPlannerConfigurator);
  ar& BOOST_SERIALIZATION_NVP(range);
}

ompl::base::PlannerPtr ESTConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::EST>(si);
  planner->setRange(range);
  planner->setGoalBias(goal_bias);
  return planner;
}

OMPLPlannerType ESTConfigurator::getType() const { return OMPLPlannerType::EST; }

template <class Archive>
void ESTConfigurator::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(OMPLPlannerConfigurator);
  ar& BOOST_SERIALIZATION_NVP(range);
  ar& BOOST_SERIALIZATION_NVP(goal_bias);
}

ompl::base::PlannerPtr LBKPIECE1Configurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::LBKPIECE1>(si);
  planner->setRange(range);
  planner->setBorderFraction(border_fraction);
  planner->setMinValidPathFraction(min_valid_path_fraction);
  return planner;
}

OMPLPlannerType LBKPIECE1Configurator::getType() const { return OMPLPlannerType::LBKPIECE1; }

template <class Archive>
void LBKPIECE1Configurator::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(OMPLPlannerConfigurator);
  ar& BOOST_SERIALIZATION_NVP(range);
  ar& BOOST_SERIALIZATION_NVP(border_fraction);
  ar& BOOST_SERIALIZATION_NVP(min_valid_path_fraction);
}

ompl::base::PlannerPtr BKPIECE1Configurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::BKPIECE1>(si);
  planner->setRange(range);
  planner->setBorderFraction(border_fraction);
  planner->setFailedExpansionCellScoreFactor(failed_expansion_score_factor);
  planner->setMinValidPathFraction(min_valid_path_fraction);
  return planner;
}

OMPLPlannerType BKPIECE1Configurator::getType() const { return OMPLPlannerType::BKPIECE1; }

template <class Archive>
void BKPIECE1Configurator::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(OMPLPlannerConfigurator);
  ar& BOOST_SERIALIZATION_NVP(range);
  ar& BOOST_SERIALIZATION_NVP(border_fraction);
  ar& BOOST_SERIALIZATION_NVP(failed_expansion_score_factor);
  ar& BOOST_SERIALIZATION_NVP(min_valid_path_fraction);
}

ompl::base::PlannerPtr KPIECE1Configurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::KPIECE1>(si);
  planner->setRange(range);
  planner->setGoalBias(goal_bias);
  planner->setBorderFraction(border_fraction);
  planner->setFailedExpansionCellScoreFactor(failed_expansion_score_factor);
  planner->setMinValidPathFraction(min_valid_path_fraction);
  return planner;
}

OMPLPlannerType KPIECE1Configurator::getType() const { return OMPLPlannerType::KPIECE1; }

template <class Archive>
void KPIECE1Configurator::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(OMPLPlannerConfigurator);
  ar& BOOST_SERIALIZATION_NVP(range);
  ar& BOOST_SERIALIZATION_NVP(goal_bias);
  ar& BOOST_SERIALIZATION_NVP(border_fraction);
  ar& BOOST_SERIALIZATION_NVP(failed_expansion_score_factor);
  ar& BOOST_SERIALIZATION_NVP(min_valid_path_fraction);
}

ompl::base::PlannerPtr BiTRRTConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::BiTRRT>(si);
  planner->setRange(range);
  planner->setTempChangeFactor(temp_change_factor);
  planner->setCostThreshold(cost_threshold);
  planner->setInitTemperature(init_temperature);
  planner->setFrontierThreshold(frontier_threshold);
  planner->setFrontierNodeRatio(frontier_node_ratio);
  return planner;
}

OMPLPlannerType BiTRRTConfigurator::getType() const { return OMPLPlannerType::BiTRRT; }

template <class Archive>
void BiTRRTConfigurator::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(OMPLPlannerConfigurator);
  ar& BOOST_SERIALIZATION_NVP(range);
  ar& BOOST_SERIALIZATION_NVP(temp_change_factor);
  ar& BOOST_SERIALIZATION_NVP(cost_threshold);
  ar& BOOST_SERIALIZATION_NVP(init_temperature);
  ar& BOOST_SERIALIZATION_NVP(frontier_threshold);
  ar& BOOST_SERIALIZATION_NVP(frontier_node_ratio);
}

ompl::base::PlannerPtr RRTConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::RRT>(si);
  planner->setRange(range);
  planner->setGoalBias(goal_bias);
  return planner;
}

OMPLPlannerType RRTConfigurator::getType() const { return OMPLPlannerType::RRT; }

template <class Archive>
void RRTConfigurator::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(OMPLPlannerConfigurator);
  ar& BOOST_SERIALIZATION_NVP(range);
  ar& BOOST_SERIALIZATION_NVP(goal_bias);
}

ompl::base::PlannerPtr RRTConnectConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::RRTConnect>(si);
  planner->setRange(range);
  return planner;
}

OMPLPlannerType RRTConnectConfigurator::getType() const { return OMPLPlannerType::RRTConnect; }

template <class Archive>
void RRTConnectConfigurator::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(OMPLPlannerConfigurator);
  ar& BOOST_SERIALIZATION_NVP(range);
}

ompl::base::PlannerPtr RRTstarConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::RRTstar>(si);
  planner->setRange(range);
  planner->setGoalBias(goal_bias);
  planner->setDelayCC(delay_collision_checking);
  return planner;
}

OMPLPlannerType RRTstarConfigurator::getType() const { return OMPLPlannerType::RRTstar; }

template <class Archive>
void RRTstarConfigurator::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(OMPLPlannerConfigurator);
  ar& BOOST_SERIALIZATION_NVP(range);
  ar& BOOST_SERIALIZATION_NVP(goal_bias);
  ar& BOOST_SERIALIZATION_NVP(delay_collision_checking);
}

ompl::base::PlannerPtr TRRTConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::TRRT>(si);
  planner->setRange(range);
  planner->setGoalBias(goal_bias);
  planner->setTempChangeFactor(temp_change_factor);
  planner->setInitTemperature(init_temperature);
  planner->setFrontierThreshold(frontier_threshold);
  planner->setFrontierNodeRatio(frontier_node_ratio);
  return planner;
}

OMPLPlannerType TRRTConfigurator::getType() const { return OMPLPlannerType::TRRT; }

template <class Archive>
void TRRTConfigurator::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(OMPLPlannerConfigurator);
  ar& BOOST_SERIALIZATION_NVP(range);
  ar& BOOST_SERIALIZATION_NVP(goal_bias);
  ar& BOOST_SERIALIZATION_NVP(temp_change_factor);
  ar& BOOST_SERIALIZATION_NVP(init_temperature);
  ar& BOOST_SERIALIZATION_NVP(frontier_threshold);
  ar& BOOST_SERIALIZATION_NVP(frontier_node_ratio);
}

ompl::base::PlannerPtr PRMConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::PRM>(si);
  planner->setMaxNearestNeighbors(static_cast<unsigned>(max_nearest_neighbors));
  return planner;
}

OMPLPlannerType PRMConfigurator::getType() const { return OMPLPlannerType::PRM; }

template <class Archive>
void PRMConfigurator::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(OMPLPlannerConfigurator);
  ar& BOOST_SERIALIZATION_NVP(max_nearest_neighbors);
}

ompl::base::PlannerPtr PRMstarConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  return std::make_shared<ompl::geometric::PRMstar>(si);
}

OMPLPlannerType PRMstarConfigurator::getType() const { return OMPLPlannerType::PRMstar; }

template <class Archive>
void PRMstarConfigurator::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(OMPLPlannerConfigurator);
}

ompl::base::PlannerPtr LazyPRMstarConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  return std::make_shared<ompl::geometric::LazyPRMstar>(si);
}

OMPLPlannerType LazyPRMstarConfigurator::getType() const { return OMPLPlannerType::LazyPRMstar; }

template <class Archive>
void LazyPRMstarConfigurator::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(OMPLPlannerConfigurator);
}

ompl::base::PlannerPtr SPARSConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::SPARS>(si);
  planner->setMaxFailures(static_cast<unsigned>(max_failures));
  planner->setDenseDeltaFraction(dense_delta_fraction);
  planner->setSparseDeltaFraction(sparse_delta_fraction);
  planner->setStretchFactor(stretch_factor);
  return planner;
}

OMPLPlannerType SPARSConfigurator::getType() const { return OMPLPlannerType::SPARS; }

template <class Archive>
void SPARSConfigurator::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(OMPLPlannerConfigurator);
  ar& BOOST_SERIALIZATION_NVP(max_failures);
  ar& BOOST_SERIALIZATION_NVP(dense_delta_fraction);
  ar& BOOST_SERIALIZATION_NVP(sparse_delta_fraction);
  ar& BOOST_SERIALIZATION_NVP(stretch_factor);
}
}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::OMPLPlannerConfigurator)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::SBLConfigurator)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::ESTConfigurator)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::LBKPIECE1Configurator)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::BKPIECE1Configurator)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::KPIECE1Configurator)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::BiTRRTConfigurator)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::RRTConfigurator)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::RRTConnectConfigurator)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::RRTstarConfigurator)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TRRTConfigurator)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::PRMConfigurator)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::PRMstarConfigurator)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::LazyPRMstarConfigurator)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::SPARSConfigurator)

BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::OMPLPlannerConfigurator)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::SBLConfigurator)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::ESTConfigurator)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::LBKPIECE1Configurator)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::BKPIECE1Configurator)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::KPIECE1Configurator)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::BiTRRTConfigurator)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::RRTConfigurator)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::RRTConnectConfigurator)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::RRTstarConfigurator)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TRRTConfigurator)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::PRMConfigurator)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::PRMstarConfigurator)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::LazyPRMstarConfigurator)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::SPARSConfigurator)
