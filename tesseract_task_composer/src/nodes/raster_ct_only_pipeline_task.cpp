/**
 * @file raster_ct_only_pipeline_task.h
 * @brief Raster only motion planning task with cartesian transitions
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <boost/serialization/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_common/timer.h>

#include <tesseract_task_composer/nodes/simple_motion_pipeline_task.h>
#include <tesseract_task_composer/nodes/raster_ct_only_pipeline_task.h>
#include <tesseract_task_composer/nodes/raster_ct_only_motion_task.h>
#include <tesseract_task_composer/task_composer_future.h>
#include <tesseract_task_composer/task_composer_executor.h>
#include <tesseract_command_language/composite_instruction.h>

namespace tesseract_planning
{
RasterCtOnlyPipelineTask::RasterCtOnlyPipelineTask(std::string input_key, std::string output_key, std::string name)
  : TaskComposerGraph(std::move(name))
{
  input_keys_.push_back(std::move(input_key));
  output_keys_.push_back(std::move(output_key));

  // Simple Motion Pipeline
  auto simple_task = std::make_unique<SimpleMotionPipelineTask>(input_keys_[0], output_keys_[0]);
  auto simple_uuid = addNode(std::move(simple_task));

  // Raster planner
  auto raster_task = std::make_unique<RasterCtOnlyMotionTask>(output_keys_[0], output_keys_[0], false);
  auto raster_uuid = addNode(std::move(raster_task));

  addEdges(simple_uuid, { raster_uuid });
}

TaskComposerNode::UPtr RasterCtOnlyPipelineTask::clone() const
{
  return std::make_unique<RasterCtOnlyPipelineTask>(input_keys_[0], output_keys_[0], name_);
}

bool RasterCtOnlyPipelineTask::operator==(const RasterCtOnlyPipelineTask& rhs) const
{
  bool equal = true;
  equal &= TaskComposerGraph::operator==(rhs);
  return equal;
}
bool RasterCtOnlyPipelineTask::operator!=(const RasterCtOnlyPipelineTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void RasterCtOnlyPipelineTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerGraph);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::RasterCtOnlyPipelineTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::RasterCtOnlyPipelineTask)
