/**
 * @file process_planning_future.cpp
 * @brief A process planning future
 *
 * @author Levi Armstrong
 * @date August 18, 2020
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/unique_ptr.hpp>
#include <boost/serialization/shared_ptr.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/process_planning_future.h>

namespace tesseract_planning
{
void ProcessPlanningFuture::clear()
{
  interface = nullptr;
  input = nullptr;
  results = nullptr;
  global_manip_info = nullptr;
  plan_profile_remapping = nullptr;
  composite_profile_remapping = nullptr;
  taskflow_container.clear();
}

bool ProcessPlanningFuture::ready() const
{
  return (process_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready);
}

void ProcessPlanningFuture::wait() const { process_future.wait(); }

std::future_status ProcessPlanningFuture::waitFor(const std::chrono::duration<double>& duration) const
{
  return process_future.wait_for(duration);
}

std::future_status
ProcessPlanningFuture::waitUntil(const std::chrono::time_point<std::chrono::high_resolution_clock>& abs) const
{
  return process_future.wait_until(abs);
}

bool ProcessPlanningFuture::operator==(const tesseract_planning::ProcessPlanningFuture& rhs) const
{
  bool equal = true;
  equal &= process_future.valid() == rhs.process_future.valid();
  equal &= (input && rhs.input && *input == *rhs.input) || (!input && !rhs.input);
  equal &= (results && rhs.results && *results == *rhs.results) || (!results && !rhs.results);
  equal &= (global_manip_info && rhs.global_manip_info && *global_manip_info == *rhs.global_manip_info) ||
           (!global_manip_info && !rhs.global_manip_info);
  equal &= (plan_profile_remapping && rhs.plan_profile_remapping &&
            *plan_profile_remapping == *rhs.plan_profile_remapping) ||
           (!plan_profile_remapping && !rhs.plan_profile_remapping);
  equal &= (composite_profile_remapping && rhs.composite_profile_remapping &&
            *composite_profile_remapping == *rhs.composite_profile_remapping) ||
           (!composite_profile_remapping && !rhs.composite_profile_remapping);
  equal &= tesseract_common::pointersEqual(interface, rhs.interface);

  return equal;
}

bool ProcessPlanningFuture::operator!=(const tesseract_planning::ProcessPlanningFuture& rhs) const
{
  return !operator==(rhs);
}

template <class Archive>
void ProcessPlanningFuture::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(input);
  ar& BOOST_SERIALIZATION_NVP(results);
  ar& BOOST_SERIALIZATION_NVP(global_manip_info);
  ar& BOOST_SERIALIZATION_NVP(plan_profile_remapping);
  ar& BOOST_SERIALIZATION_NVP(composite_profile_remapping);
  ar& BOOST_SERIALIZATION_NVP(interface);
  // These are not currently serializable
  //  ar& BOOST_SERIALIZATION_NVP(process_future);
  //  ar& BOOST_SERIALIZATION_NVP(taskflow_container);
}

}  // namespace tesseract_planning
#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::ProcessPlanningFuture)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::ProcessPlanningFuture)
