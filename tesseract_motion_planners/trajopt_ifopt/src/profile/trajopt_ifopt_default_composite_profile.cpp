/**
 * @file trajopt_default_composite_profile.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date June 18, 2020
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
#include <tinyxml2.h>
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>
#include <trajopt_common/collision_types.h>
#include <trajopt_common/utils.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/shared_ptr.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_problem.h>
#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_utils.h>

#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/eigen_serialization.h>
#include <tesseract_collision/core/serialization.h>

namespace tesseract_planning
{
TrajOptIfoptDefaultCompositeProfile::TrajOptIfoptDefaultCompositeProfile()
  : collision_cost_config(std::make_shared<trajopt_common::TrajOptCollisionConfig>())
  , collision_constraint_config(std::make_shared<trajopt_common::TrajOptCollisionConfig>())
{
}
TrajOptIfoptDefaultCompositeProfile::TrajOptIfoptDefaultCompositeProfile(const tinyxml2::XMLElement& /*xml_element*/)
  : collision_cost_config(std::make_shared<trajopt_common::TrajOptCollisionConfig>())
  , collision_constraint_config(std::make_shared<trajopt_common::TrajOptCollisionConfig>())
{
}

void TrajOptIfoptDefaultCompositeProfile::apply(TrajOptIfoptProblem& problem,
                                                int start_index,
                                                int end_index,
                                                const tesseract_common::ManipulatorInfo& manip_info,
                                                const std::vector<std::string>& /*active_links*/,
                                                const std::vector<int>& fixed_indices) const
{
  if (manip_info.manipulator.empty())
    throw std::runtime_error("TrajOptIfoptDefaultCompositeProfile, manipulator is empty!");

  if (manip_info.tcp_frame.empty())
    throw std::runtime_error("TrajOptIfoptDefaultCompositeProfile, tcp_frame is empty!");

  if (manip_info.working_frame.empty())
    throw std::runtime_error("TrajOptIfoptDefaultCompositeProfile, working_frame is empty!");

  if (start_index < 0 || start_index > static_cast<int>(problem.vars.size()) - 1)
    throw std::runtime_error("TrajOptIfoptDefaultCompositeProfile: Start index out of bounds.");

  if (end_index < 0 || end_index > static_cast<int>(problem.vars.size()) - 1)
    throw std::runtime_error("TrajOptIfoptDefaultCompositeProfile: End index out of bounds.");

  const std::vector<trajopt_ifopt::JointPosition::ConstPtr> vars(problem.vars.begin() + start_index,
                                                                 problem.vars.begin() + end_index + 1);

  if (collision_constraint_config != nullptr)
    addCollisionConstraint(
        *problem.nlp, vars, problem.environment, manip_info, collision_constraint_config, fixed_indices);

  if (collision_cost_config != nullptr)
    addCollisionCost(*problem.nlp, vars, problem.environment, manip_info, collision_cost_config, fixed_indices);

  if (smooth_velocities)
    addJointVelocitySquaredCost(*problem.nlp, vars, velocity_coeff);

  if (smooth_accelerations)
    addJointAccelerationSquaredCost(*problem.nlp, vars, acceleration_coeff);

  if (smooth_jerks)
    addJointJerkSquaredCost(*problem.nlp, vars, jerk_coeff);
}

tinyxml2::XMLElement* TrajOptIfoptDefaultCompositeProfile::toXML(tinyxml2::XMLDocument& /*doc*/) const
{
  throw std::runtime_error("TrajOptIfoptDefaultCompositeProfile::toXML is not implemented!");
}

template <class Archive>
void TrajOptIfoptDefaultCompositeProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TrajOptIfoptCompositeProfile);
  ar& BOOST_SERIALIZATION_NVP(collision_cost_config);
  ar& BOOST_SERIALIZATION_NVP(collision_constraint_config);
  ar& BOOST_SERIALIZATION_NVP(smooth_velocities);
  ar& BOOST_SERIALIZATION_NVP(velocity_coeff);
  ar& BOOST_SERIALIZATION_NVP(smooth_accelerations);
  ar& BOOST_SERIALIZATION_NVP(acceleration_coeff);
  ar& BOOST_SERIALIZATION_NVP(smooth_jerks);
  ar& BOOST_SERIALIZATION_NVP(jerk_coeff);
  ar& BOOST_SERIALIZATION_NVP(longest_valid_segment_fraction);
  ar& BOOST_SERIALIZATION_NVP(longest_valid_segment_length);
  ar& BOOST_SERIALIZATION_NVP(special_collision_cost);
  ar& BOOST_SERIALIZATION_NVP(special_collision_constraint);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TrajOptIfoptDefaultCompositeProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TrajOptIfoptDefaultCompositeProfile)
