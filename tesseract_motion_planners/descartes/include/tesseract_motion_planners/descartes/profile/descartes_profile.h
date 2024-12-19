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
#include <tesseract_kinematics/core/fwd.h>
#include <tesseract_environment/fwd.h>

#include <tesseract_command_language/profile.h>
#include <descartes_light/core/solver.h>
#include <descartes_light/core/waypoint_sampler.h>
#include <descartes_light/core/edge_evaluator.h>
#include <descartes_light/core/state_evaluator.h>

namespace tesseract_planning
{
template <typename FloatType>
class DescartesSolverProfile : public Profile
{
public:
  using Ptr = std::shared_ptr<DescartesSolverProfile<FloatType>>;
  using ConstPtr = std::shared_ptr<const DescartesSolverProfile<FloatType>>;

  DescartesSolverProfile() : Profile(DescartesSolverProfile<FloatType>::getStaticKey()) {}

  /**
   * @brief A utility function for getting profile ID
   * @return The profile ID used when storing in profile dictionary
   */
  static std::size_t getStaticKey() { return std::type_index(typeid(DescartesSolverProfile<FloatType>)).hash_code(); }

  virtual std::unique_ptr<descartes_light::Solver<FloatType>> create() const = 0;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int);  // NOLINT
};

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

  std::shared_ptr<const tesseract_kinematics::KinematicGroup>
  createKinematicGroup(const tesseract_common::ManipulatorInfo& manip_info,
                       const tesseract_environment::Environment& env) const;

  virtual std::unique_ptr<descartes_light::WaypointSampler<FloatType>>
  createWaypointSampler(const MoveInstructionPoly& move_instruction,
                        const tesseract_common::ManipulatorInfo& manip_info,
                        const std::shared_ptr<const tesseract_environment::Environment>& env) const = 0;

  virtual std::unique_ptr<descartes_light::EdgeEvaluator<FloatType>>
  createEdgeEvaluator(const MoveInstructionPoly& move_instruction,
                      const tesseract_common::ManipulatorInfo& manip_info,
                      const std::shared_ptr<const tesseract_environment::Environment>& env) const = 0;

  virtual std::unique_ptr<descartes_light::StateEvaluator<FloatType>>
  createStateEvaluator(const MoveInstructionPoly& move_instruction,
                       const tesseract_common::ManipulatorInfo& manip_info,
                       const std::shared_ptr<const tesseract_environment::Environment>& env) const = 0;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int);  // NOLINT
};

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_DESCARTES_PROFILE_H
