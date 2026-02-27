/**
 * @file ompl_planner_configurator.cpp
 * @brief Tesseract OMPL planner configurators
 *
 * @author Levi Armstrong
 * @date February 1, 2020
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

#include <tesseract/common/macros.h>
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
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/ompl_planner_configurator.h>
#include <tesseract/common/utils.h>

namespace tesseract::motion_planners
{
ompl::base::PlannerPtr SBLConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::SBL>(si);
  planner->setRange(range);
  return planner;
}

OMPLPlannerType SBLConfigurator::getType() const { return OMPLPlannerType::SBL; }

ompl::base::PlannerPtr ESTConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::EST>(si);
  planner->setRange(range);
  planner->setGoalBias(goal_bias);
  return planner;
}

OMPLPlannerType ESTConfigurator::getType() const { return OMPLPlannerType::EST; }

ompl::base::PlannerPtr LBKPIECE1Configurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::LBKPIECE1>(si);
  planner->setRange(range);
  planner->setBorderFraction(border_fraction);
  planner->setMinValidPathFraction(min_valid_path_fraction);
  return planner;
}

OMPLPlannerType LBKPIECE1Configurator::getType() const { return OMPLPlannerType::LBKPIECE1; }

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

ompl::base::PlannerPtr RRTConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::RRT>(si);
  planner->setRange(range);
  planner->setGoalBias(goal_bias);
  return planner;
}

OMPLPlannerType RRTConfigurator::getType() const { return OMPLPlannerType::RRT; }

ompl::base::PlannerPtr RRTConnectConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::RRTConnect>(si);
  planner->setRange(range);
  return planner;
}

OMPLPlannerType RRTConnectConfigurator::getType() const { return OMPLPlannerType::RRTConnect; }

ompl::base::PlannerPtr RRTstarConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::RRTstar>(si);
  planner->setRange(range);
  planner->setGoalBias(goal_bias);
  planner->setDelayCC(delay_collision_checking);
  return planner;
}

OMPLPlannerType RRTstarConfigurator::getType() const { return OMPLPlannerType::RRTstar; }

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

ompl::base::PlannerPtr PRMConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::PRM>(si);
  planner->setMaxNearestNeighbors(static_cast<unsigned>(max_nearest_neighbors));
  return planner;
}

OMPLPlannerType PRMConfigurator::getType() const { return OMPLPlannerType::PRM; }

ompl::base::PlannerPtr PRMstarConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  return std::make_shared<ompl::geometric::PRMstar>(si);
}

OMPLPlannerType PRMstarConfigurator::getType() const { return OMPLPlannerType::PRMstar; }

ompl::base::PlannerPtr LazyPRMstarConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  return std::make_shared<ompl::geometric::LazyPRMstar>(si);
}

OMPLPlannerType LazyPRMstarConfigurator::getType() const { return OMPLPlannerType::LazyPRMstar; }

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

}  // namespace tesseract::motion_planners
