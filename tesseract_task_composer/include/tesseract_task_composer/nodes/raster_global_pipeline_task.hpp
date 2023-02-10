/**
 * @file raster_global_pipeline_task.h
 * @brief A raster global planner template
 *
 * @author Matthew Powelson
 * @date July 15, 2020
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
#ifndef TESSERACT_TASK_COMPOSER_RASTER_GLOBAL_PIPELINE_TASK_HPP
#define TESSERACT_TASK_COMPOSER_RASTER_GLOBAL_PIPELINE_TASK_HPP

#include <tesseract_task_composer/task_composer_graph.h>

namespace tesseract_planning
{
template <typename SimpleTaskType, typename GlobalTaskType, typename RasterTaskType>
class RasterGlobalPipelineTask : public TaskComposerGraph
{
public:
  using class_type = RasterGlobalPipelineTask<SimpleTaskType, GlobalTaskType, RasterTaskType>;

  RasterGlobalPipelineTask() = default;  // Required for serialization
  // NOLINTNEXTLINE(performance-unnecessary-value-param)
  RasterGlobalPipelineTask(std::string input_key, std::string output_key, std::string name)
    : TaskComposerGraph(std::move(name))
  {
    input_keys_.push_back(std::move(input_key));
    output_keys_.push_back(std::move(output_key));

    // Simple Motion Pipeline
    auto simple_task = std::make_unique<SimpleTaskType>(input_keys_[0], output_keys_[0]);
    auto simple_uuid = addNode(std::move(simple_task));

    // Descartes global plan
    auto global_task = std::make_unique<GlobalTaskType>(output_keys_[0], output_keys_[0]);
    auto global_uuid = addNode(std::move(global_task));

    // Raster planner
    auto raster_task = std::make_unique<RasterTaskType>(output_keys_[0], output_keys_[0], false);
    auto raster_uuid = addNode(std::move(raster_task));

    addEdges(simple_uuid, { global_uuid });
    addEdges(global_uuid, { raster_uuid });
  }

  ~RasterGlobalPipelineTask() override = default;
  RasterGlobalPipelineTask(const class_type&) = delete;
  RasterGlobalPipelineTask& operator=(const class_type&) = delete;
  RasterGlobalPipelineTask(class_type&&) = delete;
  RasterGlobalPipelineTask& operator=(class_type&&) = delete;

  bool operator==(const class_type& rhs) const
  {
    bool equal = true;
    equal &= TaskComposerGraph::operator==(rhs);
    return equal;
  }
  bool operator!=(const class_type& rhs) const { return !operator==(rhs); }

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
  {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerGraph);
  }
};

}  // namespace tesseract_planning

#endif  // TESSERACT_TASK_COMPOSER_RASTER_GLOBAL_PIPELINE_TASK_HPP
