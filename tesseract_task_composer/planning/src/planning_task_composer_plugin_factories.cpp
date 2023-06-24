/**
 * @file task_composer_task_plugin_factory.cpp
 * @brief A task in the pipeline
 *
 * @author Levi Armstrong
 * @date July 29. 2022
 * @version TODO
 * @bug No known bugs
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

#include <tesseract_task_composer/core/task_composer_plugin_factory_utils.h>

#include <tesseract_task_composer/planning/nodes/check_input_task.h>
#include <tesseract_task_composer/planning/nodes/continuous_contact_check_task.h>
#include <tesseract_task_composer/planning/nodes/discrete_contact_check_task.h>
#include <tesseract_task_composer/planning/nodes/fix_state_bounds_task.h>
#include <tesseract_task_composer/planning/nodes/fix_state_collision_task.h>
#include <tesseract_task_composer/planning/nodes/format_as_input_task.h>
#include <tesseract_task_composer/planning/nodes/iterative_spline_parameterization_task.h>
#include <tesseract_task_composer/planning/nodes/min_length_task.h>
#include <tesseract_task_composer/planning/nodes/profile_switch_task.h>
#include <tesseract_task_composer/planning/nodes/ruckig_trajectory_smoothing_task.h>
#include <tesseract_task_composer/planning/nodes/time_optimal_parameterization_task.h>
#include <tesseract_task_composer/planning/nodes/upsample_trajectory_task.h>
#include <tesseract_task_composer/planning/nodes/raster_motion_task.h>
#include <tesseract_task_composer/planning/nodes/raster_only_motion_task.h>
#include <tesseract_task_composer/planning/nodes/motion_planner_task.hpp>
#include <tesseract_task_composer/planning/factories/planning_task_composer_plugin_factories.h>

#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/simple/simple_motion_planner.h>

#ifdef TESSERACT_TASK_COMPOSER_HAS_TRAJOPT_IFOPT
#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_motion_planner.h>
#endif

namespace tesseract_planning
{
using CheckInputTaskFactory = TaskComposerTaskFactory<CheckInputTask>;
using ContinuousContactCheckTaskFactory = TaskComposerTaskFactory<ContinuousContactCheckTask>;
using DiscreteContactCheckTaskFactory = TaskComposerTaskFactory<DiscreteContactCheckTask>;
using FixStateBoundsTaskFactory = TaskComposerTaskFactory<FixStateBoundsTask>;
using FixStateCollisionTaskFactory = TaskComposerTaskFactory<FixStateCollisionTask>;
using FormatAsInputTaskFactory = TaskComposerTaskFactory<FormatAsInputTask>;
using IterativeSplineParameterizationTaskFactory = TaskComposerTaskFactory<IterativeSplineParameterizationTask>;
using MinLengthTaskFactory = TaskComposerTaskFactory<MinLengthTask>;
using ProfileSwitchTaskFactory = TaskComposerTaskFactory<ProfileSwitchTask>;
using RuckigTrajectorySmoothingTaskFactory = TaskComposerTaskFactory<RuckigTrajectorySmoothingTask>;
using TimeOptimalParameterizationTaskFactory = TaskComposerTaskFactory<TimeOptimalParameterizationTask>;
using UpsampleTrajectoryTaskFactory = TaskComposerTaskFactory<UpsampleTrajectoryTask>;
using RasterMotionTaskFactory = TaskComposerTaskFactory<RasterMotionTask>;
using RasterOnlyMotionTaskFactory = TaskComposerTaskFactory<RasterOnlyMotionTask>;

using DescartesFMotionPlannerTaskFactory = TaskComposerTaskFactory<MotionPlannerTask<DescartesMotionPlannerF>>;
using DescartesDMotionPlannerTaskFactory = TaskComposerTaskFactory<MotionPlannerTask<DescartesMotionPlannerD>>;
using OMPLMotionPlannerTaskFactory = TaskComposerTaskFactory<MotionPlannerTask<OMPLMotionPlanner>>;
using TrajOptMotionPlannerTaskFactory = TaskComposerTaskFactory<MotionPlannerTask<TrajOptMotionPlanner>>;
using SimpleMotionPlannerTaskFactory = TaskComposerTaskFactory<MotionPlannerTask<SimpleMotionPlanner>>;
#ifdef TESSERACT_TASK_COMPOSER_HAS_TRAJOPT_IFOPT
using TrajOptIfoptMotionPlannerTaskFactory = TaskComposerTaskFactory<MotionPlannerTask<TrajOptIfoptMotionPlanner>>;
#endif

TESSERACT_PLUGIN_ANCHOR_IMPL(TaskComposerPlanningFactoriesAnchor);
}  // namespace tesseract_planning

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract_planning::CheckInputTaskFactory, CheckInputTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract_planning::ContinuousContactCheckTaskFactory,
                                        ContinuousContactCheckTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract_planning::DiscreteContactCheckTaskFactory,
                                        DiscreteContactCheckTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract_planning::FixStateBoundsTaskFactory, FixStateBoundsTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract_planning::FixStateCollisionTaskFactory, FixStateCollisionTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract_planning::FormatAsInputTaskFactory, FormatAsInputTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract_planning::IterativeSplineParameterizationTaskFactory,
                                        IterativeSplineParameterizationTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract_planning::MinLengthTaskFactory, MinLengthTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract_planning::ProfileSwitchTaskFactory, ProfileSwitchTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract_planning::RuckigTrajectorySmoothingTaskFactory,
                                        RuckigTrajectorySmoothingTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract_planning::TimeOptimalParameterizationTaskFactory,
                                        TimeOptimalParameterizationTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract_planning::UpsampleTrajectoryTaskFactory,
                                        UpsampleTrajectoryTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract_planning::RasterMotionTaskFactory, RasterMotionTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract_planning::RasterOnlyMotionTaskFactory, RasterOnlyMotionTaskFactory)

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract_planning::DescartesFMotionPlannerTaskFactory,
                                        DescartesFMotionPlannerTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract_planning::DescartesDMotionPlannerTaskFactory,
                                        DescartesDMotionPlannerTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract_planning::OMPLMotionPlannerTaskFactory, OMPLMotionPlannerTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract_planning::TrajOptMotionPlannerTaskFactory,
                                        TrajOptMotionPlannerTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract_planning::SimpleMotionPlannerTaskFactory,
                                        SimpleMotionPlannerTaskFactory)
#ifdef TESSERACT_TASK_COMPOSER_HAS_TRAJOPT_IFOPT
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract_planning::TrajOptIfoptMotionPlannerTaskFactory,
                                        TrajOptIfoptMotionPlannerTaskFactory)
#endif
