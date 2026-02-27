/**
 * @file ompl_profile.cpp
 * @brief Tesseract OMPL profile
 *
 * @author Levi Armstrong
 * @date June 18, 2020
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

#include <tesseract_motion_planners/ompl/profile/ompl_profile.h>
#include <yaml-cpp/yaml.h>
#include <tesseract/common/profile_plugin_factory.h>

namespace tesseract::motion_planners
{
OMPLMoveProfile::OMPLMoveProfile() : Profile(createKey<OMPLMoveProfile>()) {}
}  // namespace tesseract::motion_planners
