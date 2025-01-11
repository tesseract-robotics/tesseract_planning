/**
 * @file simple_planner_profile.h
 * @brief
 *
 * @author Matthew Powelson
 * @date July 23, 2020
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
#ifndef TESSERACT_MOTION_PLANNERS_SIMPLE_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_SIMPLE_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/fwd.h>
#include <tesseract_command_language/fwd.h>
#include <tesseract_environment/fwd.h>
#include <tesseract_motion_planners/core/fwd.h>
#include <tesseract_command_language/profile.h>

namespace tesseract_planning
{
/**
 * @brief Plan Profile for the simple planner. It defines some functions that handle each of the waypoint cases. The
 * planner then simply loops over all of the plan instructions and calls the correct function
 */
class SimplePlannerPlanProfile : public Profile
{
public:
  using Ptr = std::shared_ptr<SimplePlannerPlanProfile>;
  using ConstPtr = std::shared_ptr<const SimplePlannerPlanProfile>;

  SimplePlannerPlanProfile();

  /**
   * @brief A utility function for getting profile ID
   * @return The profile ID used when storing in profile dictionary
   */
  static std::size_t getStaticKey();

  /**
   * @brief Generate a seed for the provided base_instruction
   * @brief This should not include the prev_instruction but must include the base_instruction
   * @param prev_instruction The previous instruction
   * @param prev_seed The previous seed
   * @param base_instruction The base/current instruction to generate the seed for
   * @param next_instruction The next instruction. This will be a null instruction for the final instruction
   * @param request The planning request
   * @param global_manip_info The global manipulator information
   * @return A vector of move instrucitons
   */
  virtual std::vector<MoveInstructionPoly>
  generate(const MoveInstructionPoly& prev_instruction,
           const MoveInstructionPoly& prev_seed,
           const MoveInstructionPoly& base_instruction,
           const InstructionPoly& next_instruction,
           const std::shared_ptr<const tesseract_environment::Environment>& env,
           const tesseract_common::ManipulatorInfo& global_manip_info) const = 0;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

class SimplePlannerCompositeProfile : public Profile
{
public:
  using Ptr = std::shared_ptr<SimplePlannerCompositeProfile>;
  using ConstPtr = std::shared_ptr<const SimplePlannerCompositeProfile>;

  SimplePlannerCompositeProfile();

  /**
   * @brief A utility function for getting profile ID
   * @return The profile ID used when storing in profile dictionary
   */
  static std::size_t getStaticKey();

  // This contains functions for composite processing. Get start for example
protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::SimplePlannerPlanProfile)

#endif  // TESSERACT_MOTION_PLANNERS_SIMPLE_PROFILE_H
