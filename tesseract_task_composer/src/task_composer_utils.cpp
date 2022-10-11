/**
 * @file task_composer_utils.cpp
 * @brief A task composer utils
 *
 * @author Levi Armstrong
 * @date August 27, 2022
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2022, Levi Armstrong
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
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/task_composer_server.h>
#include <tesseract_task_composer/task_composer_utils.h>
#include <tesseract_task_composer/task_composer_node_names.h>
// Nodes
#include <tesseract_task_composer/nodes/trajopt_motion_pipeline_task.h>
#include <tesseract_task_composer/nodes/ompl_motion_pipeline_task.h>
#include <tesseract_task_composer/nodes/descartes_motion_pipeline_task.h>
#include <tesseract_task_composer/nodes/cartesian_motion_pipeline_task.h>
#include <tesseract_task_composer/nodes/freespace_motion_pipeline_task.h>
#include <tesseract_task_composer/nodes/raster_ct_global_pipeline_task.h>
#include <tesseract_task_composer/nodes/raster_ct_only_global_pipeline_task.h>
#include <tesseract_task_composer/nodes/raster_ct_only_pipeline_task.h>
#include <tesseract_task_composer/nodes/raster_ct_pipeline_task.h>
#include <tesseract_task_composer/nodes/raster_ft_global_pipeline_task.h>
#include <tesseract_task_composer/nodes/raster_ft_only_global_pipeline_task.h>
#include <tesseract_task_composer/nodes/raster_ft_only_pipeline_task.h>
#include <tesseract_task_composer/nodes/raster_ft_pipeline_task.h>
#ifdef TESSERACT_TASK_COMPOSER_HAS_TRAJOPT_IFOPT
#include <tesseract_task_composer/nodes/trajopt_ifopt_motion_pipeline_task.h>
#endif
// Task Profiles
#include <tesseract_task_composer/profiles/check_input_profile.h>
#include <tesseract_task_composer/profiles/contact_check_profile.h>
#include <tesseract_task_composer/profiles/fix_state_bounds_profile.h>
#include <tesseract_task_composer/profiles/fix_state_collision_profile.h>
#include <tesseract_task_composer/profiles/interative_spline_parameterization_profile.h>
#include <tesseract_task_composer/profiles/min_length_profile.h>
#include <tesseract_task_composer/profiles/profile_switch_profile.h>
#include <tesseract_task_composer/profiles/upsample_trajectory_profile.h>
// Planner Profiles
#include <tesseract_motion_planners/default_planner_namespaces.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_no_ik_plan_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_solver_profile.h>
#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>

namespace tesseract_planning
{
void loadDefaultTaskComposerNodes(TaskComposerServer& server,
                                  const std::string& input_key,
                                  const std::string& output_key)
{
  // This currently call registerProcessPlanner which takes a lock
  server.addTask(std::make_unique<TrajOptMotionPipelineTask>(input_key, output_key));
#ifdef TESSERACT_TASK_COMPOSER_HAS_TRAJOPT_IFOPT
  server.addTask(std::make_unique<TrajOptIfoptMotionPipelineTask>(input_key, output_key));
#endif
  server.addTask(std::make_unique<OMPLMotionPipelineTask>(input_key, output_key));
  server.addTask(std::make_unique<DescartesMotionPipelineTask>(input_key, output_key));
  server.addTask(std::make_unique<CartesianMotionPipelineTask>(input_key, output_key));
  server.addTask(std::make_unique<FreespaceMotionPipelineTask>(input_key, output_key));
  server.addTask(std::make_unique<RasterCtGlobalPipelineTask>(input_key, output_key));
  server.addTask(std::make_unique<RasterCtOnlyGlobalPipelineTask>(input_key, output_key));
  server.addTask(std::make_unique<RasterCtOnlyPipelineTask>(input_key, output_key));
  server.addTask(std::make_unique<RasterCtPipelineTask>(input_key, output_key));
  server.addTask(std::make_unique<RasterFtGlobalPipelineTask>(input_key, output_key));
  server.addTask(std::make_unique<RasterFtOnlyGlobalPipelineTask>(input_key, output_key));
  server.addTask(std::make_unique<RasterFtOnlyPipelineTask>(input_key, output_key));
  server.addTask(std::make_unique<RasterFtPipelineTask>(input_key, output_key));
}

void addDefaultTaskComposerProfiles(ProfileDictionary& profiles, const std::vector<std::string>& profile_names)
{
  auto check_input_profile = std::make_shared<CheckInputProfile>();
  auto contact_check_profile = std::make_shared<ContactCheckProfile>();
  auto fix_state_bounds_profile = std::make_shared<FixStateBoundsProfile>();
  auto fix_state_collision_profile = std::make_shared<FixStateCollisionProfile>();
  auto isp_profile = std::make_shared<IterativeSplineParameterizationProfile>();
  auto min_length_profile = std::make_shared<MinLengthProfile>();
  auto profile_switch_profile = std::make_shared<ProfileSwitchProfile>();
  auto upsample_trajectory_profile = std::make_shared<UpsampleTrajectoryProfile>();

  for (const std::string& profile_name : profile_names)
  {
    using namespace node_names;
    profiles.addProfile<CheckInputProfile>(CHECK_INPUT_TASK_NAME, profile_name, check_input_profile);
    profiles.addProfile<ContactCheckProfile>(DISCRETE_CONTACT_CHECK_TASK_NAME, profile_name, contact_check_profile);
    profiles.addProfile<FixStateBoundsProfile>(FIX_STATE_BOUNDS_TASK_NAME, profile_name, fix_state_bounds_profile);
    profiles.addProfile<FixStateCollisionProfile>(
        FIX_STATE_COLLISION_TASK_NAME, profile_name, fix_state_collision_profile);
    profiles.addProfile<IterativeSplineParameterizationProfile>(
        ITERATIVE_SPLINE_PARAMETERIZATION_TASK_NAME, profile_name, isp_profile);
    profiles.addProfile<MinLengthProfile>(MIN_LENGTH_TASK_NAME, profile_name, min_length_profile);
    profiles.addProfile<ProfileSwitchProfile>(PROFILE_SWITCH_TASK_NAME, profile_name, profile_switch_profile);
    profiles.addProfile<UpsampleTrajectoryProfile>(
        UPSAMPLE_TRAJECTORY_TASK_NAME, profile_name, upsample_trajectory_profile);
  }
}

void addDefaultPlannerProfiles(ProfileDictionary& profiles, const std::vector<std::string>& profile_names)
{
  auto simple_plan_profile = std::make_shared<SimplePlannerLVSNoIKPlanProfile>();
  auto descartes_profile = std::make_shared<DescartesDefaultPlanProfileF>();
  auto trajopt_plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
  auto trajopt_composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
  auto trajopt_solver_profile = std::make_shared<TrajOptDefaultSolverProfile>();
  auto ompl_profile = std::make_shared<OMPLDefaultPlanProfile>();

  for (const std::string& profile_name : profile_names)
  {
    using namespace profile_ns;
    profiles.addProfile<SimplePlannerPlanProfile>(SIMPLE_DEFAULT_NAMESPACE, profile_name, simple_plan_profile);
    profiles.addProfile<DescartesPlanProfile<float>>(DESCARTES_DEFAULT_NAMESPACE, profile_name, descartes_profile);
    profiles.addProfile<TrajOptPlanProfile>(TRAJOPT_DEFAULT_NAMESPACE, profile_name, trajopt_plan_profile);
    profiles.addProfile<TrajOptCompositeProfile>(TRAJOPT_DEFAULT_NAMESPACE, profile_name, trajopt_composite_profile);
    profiles.addProfile<TrajOptSolverProfile>(TRAJOPT_DEFAULT_NAMESPACE, profile_name, trajopt_solver_profile);
    profiles.addProfile<OMPLPlanProfile>(OMPL_DEFAULT_NAMESPACE, profile_name, ompl_profile);
  }
}

}  // namespace tesseract_planning
