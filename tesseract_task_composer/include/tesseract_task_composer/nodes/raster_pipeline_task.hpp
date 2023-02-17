/**
 * @file raster_pipeline_task.h
 * @brief Plans raster paths
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
#ifndef TESSERACT_TASK_COMPOSER_RASTER_PIPELINE_TASK_HPP
#define TESSERACT_TASK_COMPOSER_RASTER_PIPELINE_TASK_HPP

#include <tesseract_task_composer/task_composer_graph.h>

namespace tesseract_planning
{
template <typename SimpleTaskType, typename RasterTaskType>
class RasterPipelineTask : public TaskComposerGraph
{
public:
  using class_type = RasterPipelineTask<SimpleTaskType, RasterTaskType>;

  RasterPipelineTask() = default;  // Required for serialization
  // NOLINTNEXTLINE(performance-unnecessary-value-param)
  RasterPipelineTask(std::string input_key, std::string output_key, std::string name)
    : TaskComposerGraph(std::move(name))
  {
    input_keys_.push_back(std::move(input_key));
    output_keys_.push_back(std::move(output_key));

    // Simple Motion Pipeline
    auto simple_task = std::make_unique<SimpleTaskType>(input_keys_[0], output_keys_[0]);
    auto simple_uuid = addNode(std::move(simple_task));

    // Raster planner
    auto raster_task = std::make_unique<RasterTaskType>(output_keys_[0], output_keys_[0], false);
    auto raster_uuid = addNode(std::move(raster_task));

    addEdges(simple_uuid, { raster_uuid });
  }
  ~RasterPipelineTask() override = default;
  RasterPipelineTask(const class_type&) = delete;
  RasterPipelineTask& operator=(const class_type&) = delete;
  RasterPipelineTask(class_type&&) = delete;
  RasterPipelineTask& operator=(class_type&&) = delete;

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

#endif  // TESSERACT_TASK_COMPOSER_RASTER_CT_PIPELINE_TASK_H
