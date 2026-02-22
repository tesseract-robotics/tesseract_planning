/**
 * @file task_composer_task_plugin_factory.cpp
 * @brief A task in the pipeline
 *
 * @author Levi Armstrong
 * @date July 29. 2022
 *
 * @copyright Copyright (c) 2023, Levi Armstrong
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

#include <tesseract_task_composer/planning/planning_task_composer_plugin_factories.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory_utils.h>

#include <tesseract_task_composer/planning/nodes/continuous_contact_check_task.h>
#include <tesseract_task_composer/planning/nodes/discrete_contact_check_task.h>
#include <tesseract_task_composer/planning/nodes/fix_state_bounds_task.h>
#include <tesseract_task_composer/planning/nodes/fix_state_collision_task.h>
#include <tesseract_task_composer/planning/nodes/format_as_input_task.h>
#include <tesseract_task_composer/planning/nodes/format_as_result_task.h>
#include <tesseract_task_composer/planning/nodes/format_planning_input_task.h>
#include <tesseract_task_composer/planning/nodes/kinematic_limits_check_task.h>
#include <tesseract_task_composer/planning/nodes/min_length_task.h>
#include <tesseract_task_composer/planning/nodes/profile_switch_task.h>
#include <tesseract_task_composer/planning/nodes/upsample_trajectory_task.h>
#include <tesseract_task_composer/planning/nodes/raster_motion_task.h>
#include <tesseract_task_composer/planning/nodes/raster_only_motion_task.h>
#include <tesseract_task_composer/planning/nodes/motion_planner_task.hpp>
#include <tesseract_task_composer/planning/nodes/process_planning_input_task.h>

#include <tesseract_motion_planners/simple/simple_motion_planner.h>

#include <boost_plugin_loader/macros.h>

namespace tesseract::task_composer
{
using ContinuousContactCheckTaskFactory = TaskComposerTaskFactory<ContinuousContactCheckTask>;
using DiscreteContactCheckTaskFactory = TaskComposerTaskFactory<DiscreteContactCheckTask>;
using FixStateBoundsTaskFactory = TaskComposerTaskFactory<FixStateBoundsTask>;
using FixStateCollisionTaskFactory = TaskComposerTaskFactory<FixStateCollisionTask>;
using FormatAsInputTaskFactory = TaskComposerTaskFactory<FormatAsInputTask>;
using FormatAsResultTaskFactory = TaskComposerTaskFactory<FormatAsResultTask>;
using FormatPlanningInputTaskFactory = TaskComposerTaskFactory<FormatPlanningInputTask>;
using KinematicLimitsCheckTaskFactory = TaskComposerTaskFactory<KinematicLimitsCheckTask>;
using MinLengthTaskFactory = TaskComposerTaskFactory<MinLengthTask>;
using ProfileSwitchTaskFactory = TaskComposerTaskFactory<ProfileSwitchTask>;
using UpsampleTrajectoryTaskFactory = TaskComposerTaskFactory<UpsampleTrajectoryTask>;
using RasterMotionTaskFactory = TaskComposerTaskFactory<RasterMotionTask>;
using RasterOnlyMotionTaskFactory = TaskComposerTaskFactory<RasterOnlyMotionTask>;
using SimpleMotionPlannerTaskFactory =
    TaskComposerTaskFactory<MotionPlannerTask<tesseract::motion_planners::SimpleMotionPlanner>>;
using ProcessPlanningInputTaskFactory = TaskComposerTaskFactory<ProcessPlanningInputTask>;

// LCOV_EXCL_START
PLUGIN_ANCHOR_IMPL(TaskComposerPlanningFactoriesAnchor)
// LCOV_EXCL_STOP

}  // namespace tesseract::task_composer

// clang-format off
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract::task_composer::ContinuousContactCheckTaskFactory, ContinuousContactCheckTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract::task_composer::DiscreteContactCheckTaskFactory, DiscreteContactCheckTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract::task_composer::FixStateBoundsTaskFactory, FixStateBoundsTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract::task_composer::FixStateCollisionTaskFactory, FixStateCollisionTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract::task_composer::FormatAsInputTaskFactory, FormatAsInputTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract::task_composer::FormatAsResultTaskFactory, FormatAsResultTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract::task_composer::FormatPlanningInputTaskFactory, FormatPlanningInputTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract::task_composer::KinematicLimitsCheckTaskFactory, KinematicLimitsCheckTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract::task_composer::MinLengthTaskFactory, MinLengthTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract::task_composer::ProfileSwitchTaskFactory, ProfileSwitchTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract::task_composer::UpsampleTrajectoryTaskFactory, UpsampleTrajectoryTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract::task_composer::RasterMotionTaskFactory, RasterMotionTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract::task_composer::RasterOnlyMotionTaskFactory, RasterOnlyMotionTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract::task_composer::SimpleMotionPlannerTaskFactory, SimpleMotionPlannerTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract::task_composer::ProcessPlanningInputTaskFactory, ProcessPlanningInputTaskFactory)
// clang-format on
