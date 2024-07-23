/**
 * @file profile_dictionary.cpp
 * @brief This is a profile dictionary for storing all profiles
 *
 * @author Levi Armstrong
 * @date December 2, 2020
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

#include <tesseract_command_language/profile_dictionary.h>
#include <boost/serialization/shared_ptr.hpp>

namespace tesseract_planning
{
void ProfileDictionary::clear()
{
  std::unique_lock lock(mutex_);
  profiles_.clear();
}

template <class Archive>
void ProfileDictionary::serialize(Archive& /*ar*/, const unsigned int /*version*/)
{
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::ProfileDictionary)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::ProfileDictionary)
TESSERACT_ANY_EXPORT_IMPLEMENT(TesseractPlanningProfileDictionarySharedPtr)
