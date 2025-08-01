/**
 * @file fix_state_bounds_profile.cpp
 * @brief Profile for process that pushes plan instructions back within joint limits
 *
 * @author Matthew Powelson
 * @date August 31. 2020
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
#include <tesseract_task_composer/planning/profiles/fix_state_bounds_profile.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <typeindex>
#include <tesseract_task_composer/core/yaml_extensions.h>
#include <yaml-cpp/yaml.h>
#include <tesseract_common/profile_plugin_factory.h>

namespace tesseract_planning
{
FixStateBoundsProfile::FixStateBoundsProfile(Settings mode) : Profile(FixStateBoundsProfile::getStaticKey()), mode(mode)
{
}

FixStateBoundsProfile::FixStateBoundsProfile(std::string name, const YAML::Node& config, const tesseract_common::ProfilePluginFactory& /*plugin_factory*/)
  : FixStateBoundsProfile()
{   
  try
  {
    if (YAML::Node n = config["mode"])
      mode = n.as<Settings>();
    if (YAML::Node n = config["max_deviation_global"])
      max_deviation_global = n.as<double>();
    if (YAML::Node n = config["upper_bounds_reduction"])
      upper_bounds_reduction = n.as<double>();
    if (YAML::Node n = config["lower_bounds_reduction"])
      lower_bounds_reduction = n.as<double>();
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error("FixStateBoundsProfile: Failed to parse yaml config! Details: " + std::string(e.what()));
  }
}

std::size_t FixStateBoundsProfile::getStaticKey() { return std::type_index(typeid(FixStateBoundsProfile)).hash_code(); }

template <class Archive>
void FixStateBoundsProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Profile);
  ar& BOOST_SERIALIZATION_NVP(mode);
  ar& BOOST_SERIALIZATION_NVP(max_deviation_global);
  ar& BOOST_SERIALIZATION_NVP(upper_bounds_reduction);
  ar& BOOST_SERIALIZATION_NVP(lower_bounds_reduction);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::FixStateBoundsProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::FixStateBoundsProfile)
