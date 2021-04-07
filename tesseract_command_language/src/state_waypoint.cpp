/**
 * @file state_waypoint.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date June 15, 2020
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

#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/waypoint_type.h>
#include <tesseract_common/utils.h>

namespace tesseract_planning
{
StateWaypoint::StateWaypoint(std::vector<std::string> joint_names, const Eigen::Ref<const Eigen::VectorXd>& position)
  : tesseract_common::JointState(std::move(joint_names), position)
{
  if (static_cast<Eigen::Index>(this->joint_names.size()) != this->position.size())
    throw std::runtime_error("StateWaypoint: joint_names is not the same size as position!");
}

void StateWaypoint::print(const std::string& prefix) const
{
  std::cout << prefix << "State WP: Pos=" << position.transpose() << std::endl;
}

bool StateWaypoint::operator==(const StateWaypoint& rhs) const
{
  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  bool equal = true;
  equal &= tesseract_common::almostEqualRelativeAndAbs(position, rhs.position, max_diff);
  equal &= tesseract_common::isIdentical(joint_names, rhs.joint_names);
  return equal;
}
bool StateWaypoint::operator!=(const StateWaypoint& rhs) const { return !operator==(rhs); }

template <class Archive>
void StateWaypoint::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("base", boost::serialization::base_object<tesseract_common::JointState>(*this));
}

}  // namespace tesseract_planning

#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
template void tesseract_planning::StateWaypoint::serialize(boost::archive::xml_oarchive& ar,
                                                           const unsigned int version);
template void tesseract_planning::StateWaypoint::serialize(boost::archive::xml_iarchive& ar,
                                                           const unsigned int version);

TESSERACT_WAYPOINT_EXPORT_IMPLEMENT(tesseract_planning::StateWaypoint);
