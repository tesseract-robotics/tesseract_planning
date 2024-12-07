/**
 * @file ompl_profile.h
 * @brief Tesseract OMPL profile
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
#ifndef TESSERACT_MOTION_PLANNERS_OMPL_OMPL_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_OMPL_OMPL_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <memory>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/types.h>

#include <tesseract_common/fwd.h>
#include <tesseract_kinematics/core/fwd.h>
#include <tesseract_environment/fwd.h>
#include <tesseract_command_language/fwd.h>

#include <tesseract_command_language/profile.h>

namespace ompl::geometric
{
class SimpleSetup;
}

namespace tesseract_planning
{
struct OMPLSolverConfig;
struct OMPLPlannerConfigurator;

class OMPLPlanProfile : public Profile
{
public:
  using Ptr = std::shared_ptr<OMPLPlanProfile>;
  using ConstPtr = std::shared_ptr<const OMPLPlanProfile>;

  OMPLPlanProfile();

  /**
   * @brief A utility function for getting profile ID
   * @return The profile ID used when storing in profile dictionary
   */
  static std::size_t getStaticKey();

  /**
   * @brief Create the OMPL Parallel Plan Solver Config
   * @return The OMPL Parallel Plan Solver Config
   */
  virtual std::unique_ptr<OMPLSolverConfig> createSolverConfig() const = 0;

  /**
   * @brief Create the state extractor
   * @return The OMPL state extractor
   */
  virtual OMPLStateExtractor createStateExtractor(const tesseract_kinematics::JointGroup& manip) const = 0;

  /**
   * @brief Create OMPL Simple Setup
   * @param start_instruction The start instruction
   * @param end_instruction The goal instruction
   * @param composite_mi The parent composite manip info
   * @param env The environment
   * @return A OMPL Simple Setup
   */
  virtual std::unique_ptr<ompl::geometric::SimpleSetup>
  createSimpleSetup(const MoveInstructionPoly& start_instruction,
                    const MoveInstructionPoly& end_instruction,
                    const tesseract_common::ManipulatorInfo& composite_mi,
                    const std::shared_ptr<const tesseract_environment::Environment>& env) const = 0;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::OMPLPlanProfile)

#endif  // TESSERACT_MOTION_PLANNERS_OMPL_OMPL_PROFILE_H
