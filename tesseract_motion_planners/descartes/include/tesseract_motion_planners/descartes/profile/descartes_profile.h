/**
 * @file descartes_profile.h
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
#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_DESCARTES_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_DESCARTES_DESCARTES_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <typeindex>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/fwd.h>
#include <tesseract_command_language/fwd.h>
#include <tesseract_command_language/profile.h>
#include <tesseract_motion_planners/descartes/descartes_problem.h>

namespace tinyxml2
{
class XMLElement;  // NOLINT
class XMLDocument;
}  // namespace tinyxml2

namespace tesseract_planning
{
template <typename FloatType>
class DescartesPlanProfile : public Profile
{
public:
  using Ptr = std::shared_ptr<DescartesPlanProfile<FloatType>>;
  using ConstPtr = std::shared_ptr<const DescartesPlanProfile<FloatType>>;

  DescartesPlanProfile() : Profile(DescartesPlanProfile<FloatType>::getStaticKey()) {}

  /**
   * @brief A utility function for getting profile ID
   * @return The profile ID used when storing in profile dictionary
   */
  static std::size_t getStaticKey() { return std::type_index(typeid(DescartesPlanProfile<FloatType>)).hash_code(); }

  virtual void apply(DescartesProblem<FloatType>& prob,
                     const Eigen::Isometry3d& cartesian_waypoint,
                     const InstructionPoly& parent_instruction,
                     const tesseract_common::ManipulatorInfo& manip_info,
                     int index) const = 0;

  virtual void apply(DescartesProblem<FloatType>& prob,
                     const Eigen::VectorXd& joint_waypoint,
                     const InstructionPoly& parent_instruction,
                     const tesseract_common::ManipulatorInfo& manip_info,
                     int index) const = 0;

  virtual tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const = 0;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int);  // NOLINT
};

/** @todo Currently descartes does not have support of composite profile everything is handled by the plan profile */
}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_DESCARTES_PROFILE_H
