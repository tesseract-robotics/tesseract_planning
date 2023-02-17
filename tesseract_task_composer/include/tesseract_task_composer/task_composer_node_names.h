/**
 * @file task_composer_node_names.h
 * @brief Contains default node names
 *
 * @author Levi Armstrong
 * @date July 29. 2022
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
#ifndef TESSERACT_TASK_COMPOSER_TASK_COMPOSER_NODE_NAMES_H
#define TESSERACT_TASK_COMPOSER_TASK_COMPOSER_NODE_NAMES_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/default_planner_namespaces.h>

namespace tesseract_planning::node_names
{
// Graph/Pipeline names
/** @brief Simple pipeline */
static const std::string SIMPLE_PIPELINE_NAME = "SimplePipeline";

/** @brief TrajOpt pipeline */
static const std::string TRAJOPT_PIPELINE_NAME = "TrajOptPipeline";

#ifdef TESSERACT_TASK_COMPOSER_HAS_TRAJOPT_IFOPT
/** @brief TrajOpt IFOPT pipeline */
static const std::string TRAJOPT_IFOPT_PIPELINE_NAME = "TrajOptIfoptPipeline";
#endif

/** @brief OMPL pipeline */
static const std::string OMPL_PIPELINE_NAME = "OMPLPipeline";

/** @brief Descartes pipeline */
static const std::string DESCARTES_PIPELINE_NAME = "DescartesPipeline";

/** @brief Descartes no post check pipeline */
static const std::string DESCARTES_NPC_PIPELINE_NAME = "DescartesNPCPipeline";

/** @brief Freespace pipeline */
static const std::string FREESPACE_PIPELINE_NAME = "FreespacePipeline";

/** @brief Cartesian pipeline */
static const std::string CARTESIAN_PIPELINE_NAME = "CartesianPipeline";

/** @brief Raster pipeline using freespace pipeline for transitions */
static const std::string RASTER_FT_PIPELINE_NAME = "RasterFtPipeline";

/** @brief Raster pipeline using cartesian pipeline for transitions */
static const std::string RASTER_CT_PIPELINE_NAME = "RasterCtPipeline";

/** @brief Raster pipeline performs global plan first then macro planning using freespace pipeline for transitions */
static const std::string RASTER_FT_G_PIPELINE_NAME = "RasterFtGlobalPipeline";

/** @brief Raster Pipeline performs global plan first then macro planning using cartesian pipeline for transitions */
static const std::string RASTER_CT_G_PIPELINE_NAME = "RasterCtGlobalPipeline";

/** @brief Raster only pipeline using freespace pipeline for transitions */
static const std::string RASTER_FT_O_PIPELINE_NAME = "RasterFtOnlyPipeline";

/** @brief Raster only pipeline using cartesian pipeline for transitions */
static const std::string RASTER_CT_O_PIPELINE_NAME = "RasterCtOnlyPipeline";

/** @brief Raster only pipeline performs global plan first then macro planning using freespace pipeline for transitions
 */
static const std::string RASTER_FT_O_G_PIPELINE_NAME = "RasterFtOnlyGlobalPipeline";

/** @brief Raster only pipeline performs global plan first then macro planning using cartesian pipeline for transitions
 */
static const std::string RASTER_CT_O_G_PIPELINE_NAME = "RasterCtOnlyGlobalPipeline";

// Motion Planner Task Names
static const std::string DESCARTES_MOTION_PLANNER_TASK_NAME = profile_ns::DESCARTES_DEFAULT_NAMESPACE;
static const std::string OMPL_MOTION_PLANNER_TASK_NAME = profile_ns::OMPL_DEFAULT_NAMESPACE;
static const std::string SIMPLE_MOTION_PLANNER_TASK_NAME = profile_ns::SIMPLE_DEFAULT_NAMESPACE;
static const std::string TRAJOPT_MOTION_PLANNER_TASK_NAME = profile_ns::TRAJOPT_DEFAULT_NAMESPACE;
#ifdef TESSERACT_TASK_COMPOSER_HAS_TRAJOPT_IFOPT
/** @brief TrajOpt IFOPT pipeline */
static const std::string TRAJOPT_IFOPT_MOTION_PLANNER_TASK_NAME = profile_ns::TRAJOPT_IFOPT_DEFAULT_NAMESPACE;
#endif

// Task Names
static const std::string CHECK_INPUT_TASK_NAME = "CheckInputTask";
static const std::string CONTINUOUS_CONTACT_CHECK_TASK_NAME = "ContinuousContactCheckTask";
static const std::string DISCRETE_CONTACT_CHECK_TASK_NAME = "DiscreteContactCheckTask";
static const std::string DONE_TASK_NAME = "DoneTask";
static const std::string ERROR_TASK_NAME = "ErrorTask";
static const std::string FIX_STATE_BOUNDS_TASK_NAME = "FixStateBoundsTask";
static const std::string FIX_STATE_COLLISION_TASK_NAME = "FixStateCollisionTask";
static const std::string ITERATIVE_SPLINE_PARAMETERIZATION_TASK_NAME = "IterativeSplineParameterizationTask";
static const std::string MIN_LENGTH_TASK_NAME = "MinLengthTask";
static const std::string PROFILE_SWITCH_TASK_NAME = "ProfileSwitchTask";
static const std::string RASTER_CT_MOTION_TASK_NAME = "RasterCtMotionTask";
static const std::string RASTER_CT_ONLY_MOTION_TASK_NAME = "RasterCtOnlyMotionTask";
static const std::string RASTER_FT_MOTION_TASK_NAME = "RasterFtMotionTask";
static const std::string RASTER_FT_ONLY_MOTION_TASK_NAME = "RasterFtOnlyMotionTask";
static const std::string RUCKIG_TRAJECTORY_SMOOTHING_TASK_NAME = "RuckigTrajectorySmoothingTask";
static const std::string START_TASK_NAME = "StartTask";
static const std::string TIME_OPTIMAL_PARAMETERIZATION_TASK_NAME = "TimeOptimalParameterizationTask";
static const std::string UPDATE_END_STATE_TASK_NAME = "UpdateEndStateTask";
static const std::string UPDATE_START_AND_END_STATE_TASK_NAME = "UpdateStartAndEndStateTask";
static const std::string UPDATE_START_STATE_TASK_NAME = "UpdateStartStateTask";
static const std::string UPSAMPLE_TRAJECTORY_TASK_NAME = "UpsampleTrajectoryTask";

}  // namespace tesseract_planning::node_names

#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_NODE_NAMES_H
