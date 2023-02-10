/**
 * @file raster_ct_motion_task.h
 * @brief Raster motion task with cartesian transitions
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
#ifndef TESSERACT_TASK_COMPOSER_RASTER_CT_MOTION_TASK_H
#define TESSERACT_TASK_COMPOSER_RASTER_CT_MOTION_TASK_H

#include <tesseract_task_composer/task_composer_task.h>
#include <tesseract_task_composer/task_composer_node_names.h>

#include <tesseract_task_composer/nodes/raster_motion_task.hpp>
#include <tesseract_task_composer/nodes/cartesian_motion_pipeline_task.h>
#include <tesseract_task_composer/nodes/freespace_motion_pipeline_task.h>

namespace tesseract_planning
{
using RasterCtMotionTaskBase =
    RasterMotionTask<FreespaceMotionPipelineTask, CartesianMotionPipelineTask, CartesianMotionPipelineTask>;
class RasterCtMotionTask : public RasterCtMotionTaskBase
{
public:
  using Ptr = std::shared_ptr<RasterCtMotionTask>;
  using ConstPtr = std::shared_ptr<const RasterCtMotionTask>;
  using UPtr = std::unique_ptr<RasterCtMotionTask>;
  using ConstUPtr = std::unique_ptr<const RasterCtMotionTask>;

  RasterCtMotionTask() = default;  // Required for serialization
  RasterCtMotionTask(std::string input_key,
                     std::string output_key,
                     bool is_conditional = true,
                     std::string name = node_names::RASTER_CT_MOTION_TASK_NAME);
  ~RasterCtMotionTask() override = default;
  RasterCtMotionTask(const RasterCtMotionTask&) = delete;
  RasterCtMotionTask& operator=(const RasterCtMotionTask&) = delete;
  RasterCtMotionTask(RasterCtMotionTask&&) = delete;
  RasterCtMotionTask& operator=(RasterCtMotionTask&&) = delete;

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::RasterCtMotionTaskBase, "RasterCtMotionTaskBase")
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::RasterCtMotionTask, "RasterCtMotionTask")

#endif  // TESSERACT_TASK_COMPOSER_RASTER_CT_MOTION_TASK_H
