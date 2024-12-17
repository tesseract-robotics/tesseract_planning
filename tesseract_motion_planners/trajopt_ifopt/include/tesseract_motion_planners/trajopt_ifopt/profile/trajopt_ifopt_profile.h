/**
 * @file trajopt_profile.h
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
#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <memory>
#include <trajopt_ifopt/fwd.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/fwd.h>
#include <tesseract_command_language/fwd.h>
#include <tesseract_command_language/profile.h>

namespace tinyxml2
{
class XMLElement;  // NOLINT
class XMLDocument;
}  // namespace tinyxml2

namespace tesseract_planning
{
struct TrajOptIfoptProblem;

class TrajOptIfoptPlanProfile : public Profile
{
public:
  using Ptr = std::shared_ptr<TrajOptIfoptPlanProfile>;
  using ConstPtr = std::shared_ptr<const TrajOptIfoptPlanProfile>;

  TrajOptIfoptPlanProfile();

  /**
   * @brief A utility function for getting profile ID
   * @return The profile ID used when storing in profile dictionary
   */
  static std::size_t getStaticKey();

  virtual void apply(TrajOptIfoptProblem& problem,
                     const CartesianWaypointPoly& cartesian_waypoint,
                     const InstructionPoly& parent_instruction,
                     const tesseract_common::ManipulatorInfo& manip_info,
                     const std::vector<std::string>& active_links,
                     int index) const = 0;

  virtual void apply(TrajOptIfoptProblem& problem,
                     const JointWaypointPoly& joint_waypoint,
                     const InstructionPoly& parent_instruction,
                     const tesseract_common::ManipulatorInfo& manip_info,
                     const std::vector<std::string>& active_links,
                     int index) const = 0;

  virtual tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const = 0;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

class TrajOptIfoptCompositeProfile : public Profile
{
public:
  using Ptr = std::shared_ptr<TrajOptIfoptCompositeProfile>;
  using ConstPtr = std::shared_ptr<const TrajOptIfoptCompositeProfile>;

  TrajOptIfoptCompositeProfile();

  /**
   * @brief A utility function for getting profile ID
   * @return The profile ID used when storing in profile dictionary
   */
  static std::size_t getStaticKey();

  virtual void apply(TrajOptIfoptProblem& problem,
                     int start_index,
                     int end_index,
                     const tesseract_common::ManipulatorInfo& manip_info,
                     const std::vector<std::string>& active_links,
                     const std::vector<int>& fixed_indices) const = 0;

  virtual tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const = 0;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

class TrajOptIfoptSolverProfile : public Profile
{
public:
  using Ptr = std::shared_ptr<TrajOptIfoptSolverProfile>;
  using ConstPtr = std::shared_ptr<const TrajOptIfoptSolverProfile>;

  TrajOptIfoptSolverProfile();

  /**
   * @brief A utility function for getting profile ID
   * @return The profile ID used when storing in profile dictionary
   */
  static std::size_t getStaticKey();

  virtual void apply(TrajOptIfoptProblem& problem) const = 0;

  virtual tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const = 0;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::TrajOptIfoptPlanProfile)
BOOST_CLASS_EXPORT_KEY(tesseract_planning::TrajOptIfoptCompositeProfile)
BOOST_CLASS_EXPORT_KEY(tesseract_planning::TrajOptIfoptSolverProfile)

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_PROFILE_H
