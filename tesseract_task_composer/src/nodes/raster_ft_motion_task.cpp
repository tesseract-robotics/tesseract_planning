/**
 * @file raster_ft_motion_task.cpp
 * @brief Raster motion planning task with freespace transitions
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

#include <tesseract_task_composer/nodes/raster_ft_motion_task.h>

namespace tesseract_planning
{
RasterFtMotionTask::RasterFtMotionTask(std::string input_key,
                                       std::string output_key,
                                       bool is_conditional,
                                       std::string name)
  : RasterFtMotionTaskBase(std::move(input_key), std::move(output_key), is_conditional, std::move(name))
{
}

template <class Archive>
void RasterFtMotionTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(RasterFtMotionTaskBase);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::RasterFtMotionTaskBase)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::RasterFtMotionTaskBase)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::RasterFtMotionTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::RasterFtMotionTask)
