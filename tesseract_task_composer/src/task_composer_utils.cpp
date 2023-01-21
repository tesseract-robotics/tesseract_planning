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

#include <tesseract_task_composer/task_composer_utils.h>
#include <tesseract_task_composer/task_composer_node_names.h>
#include <tesseract_task_composer/nodes/trajopt_motion_pipeline_task.h>
#include <tesseract_task_composer/nodes/ompl_motion_pipeline_task.h>
#include <tesseract_task_composer/nodes/descartes_motion_pipeline_task.h>
#include <tesseract_task_composer/nodes/descartes_npc_motion_pipeline_task.h>
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
  server.addTask(std::make_unique<DescartesNPCMotionPipelineTask>(input_key, output_key));
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
}  // namespace tesseract_planning
