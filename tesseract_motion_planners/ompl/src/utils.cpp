/**
 * @file utils.cpp
 * @brief Tesseract OMPL planner utility functions
 *
 * @author Levi Armstrong
 * @date February 17, 2020
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
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/State.h>
#include <ompl/geometric/PathGeometric.h>
#include <memory>

TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/utils.h>

#include <tesseract_common/types.h>

#include <tesseract_collision/core/types.h>
#include <tesseract_collision/core/discrete_contact_manager.h>

#include <tesseract_kinematics/core/joint_group.h>

namespace tesseract_planning
{
void processLongestValidSegment(const ompl::base::StateSpacePtr& state_space_ptr,
                                double longest_valid_segment_fraction,
                                double longest_valid_segment_length)
{
  if (longest_valid_segment_fraction > 0 && longest_valid_segment_length > 0)
  {
    double val =
        std::min(longest_valid_segment_fraction, longest_valid_segment_length / state_space_ptr->getMaximumExtent());
    longest_valid_segment_fraction = val;
  }
  else if (longest_valid_segment_length > 0)
  {
    longest_valid_segment_fraction = longest_valid_segment_length / state_space_ptr->getMaximumExtent();
  }
  else
  {
    longest_valid_segment_fraction = 0.01;
  }
  state_space_ptr->setLongestValidSegmentFraction(longest_valid_segment_fraction);
}

void processLongestValidSegment(const ompl::base::StateSpacePtr& state_space_ptr,
                                const tesseract_collision::CollisionCheckConfig& collision_check_config)
{
  double longest_valid_segment_fraction = 0.01;
  if (collision_check_config.longest_valid_segment_length > 0)
    longest_valid_segment_fraction =
        collision_check_config.longest_valid_segment_length / state_space_ptr->getMaximumExtent();

  state_space_ptr->setLongestValidSegmentFraction(longest_valid_segment_fraction);
}

bool checkStateInCollision(tesseract_collision::ContactResultMap& contact_map,
                           tesseract_collision::DiscreteContactManager& contact_checker,
                           const tesseract_kinematics::JointGroup& manip,
                           const Eigen::VectorXd& state)
{
  tesseract_common::TransformMap link_transforms = manip.calcFwdKin(state);

  for (const auto& link_name : contact_checker.getActiveCollisionObjects())
    contact_checker.setCollisionObjectsTransform(link_name, link_transforms[link_name]);

  contact_checker.contactTest(contact_map, tesseract_collision::ContactTestType::FIRST);

  return (!contact_map.empty());
}

// long assignTrajectory(tesseract_planning::CompositeInstruction& output,
//                       boost::uuids::uuid start_uuid,
//                       boost::uuids::uuid end_uuid,
//                       long start_index,
//                       const std::vector<std::string>& joint_names,
//                       const tesseract_common::TrajArray& traj,
//                       const bool format_result_as_input)
// {
//   bool found{ false };
//   Eigen::Index row{ 0 };
//   auto& ci = output.getInstructions();
//   for (auto it = ci.begin() + static_cast<long>(start_index); it != ci.end(); ++it)
//   {
//     if (it->isMoveInstruction())
//     {
//       auto& mi = it->as<MoveInstructionPoly>();
//       if (mi.getUUID() == start_uuid)
//         found = true;

//       if (mi.getUUID() == end_uuid)
//       {
//         std::vector<InstructionPoly> extra;
//         for (; row < traj.rows() - 1; ++row)
//         {
//           MoveInstructionPoly child = mi.createChild();
//           if (format_result_as_input)
//           {
//             JointWaypointPoly jwp = mi.createJointWaypoint();
//             jwp.setIsConstrained(false);
//             jwp.setNames(joint_names);
//             jwp.setPosition(traj.row(row));
//             child.assignJointWaypoint(jwp);
//           }
//           else
//           {
//             StateWaypointPoly swp = mi.createStateWaypoint();
//             swp.setNames(joint_names);
//             swp.setPosition(traj.row(row));
//             child.assignStateWaypoint(swp);
//           }

//           extra.emplace_back(child);
//         }

//         assignSolution(mi, joint_names, traj.row(row), format_result_as_input);

//         if (!extra.empty())
//           ci.insert(it, extra.begin(), extra.end());

//         start_index += static_cast<long>(extra.size());
//         break;
//       }

//       if (found)
//         assignSolution(mi, joint_names, traj.row(row++), format_result_as_input);
//     }

//     ++start_index;
//   }

//   return start_index;
// }

}  // namespace tesseract_planning
