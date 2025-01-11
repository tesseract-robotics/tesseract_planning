/**
 * @file ompl_real_vector_plan_profile.h
 * @brief Tesseract OMPL real vector state space plan profile
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
#ifndef TESSERACT_MOTION_PLANNERS_OMPL_OMPL_REAL_VECTOR_PLAN_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_OMPL_OMPL_REAL_VECTOR_PLAN_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <memory>
#include <functional>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/profile/ompl_profile.h>
#include <tesseract_motion_planners/ompl/ompl_solver_config.h>

#include <tesseract_collision/core/fwd.h>
#include <tesseract_collision/core/types.h>

#include <tesseract_command_language/fwd.h>
#include <tesseract_kinematics/core/fwd.h>
#include <tesseract_common/fwd.h>

namespace ompl::base
{
class StateSampler;
class StateSpace;
class StateValidityChecker;
class MotionValidator;
class OptimizationObjective;
}  // namespace ompl::base

namespace tesseract_planning
{
/**
 * @brief OMPL does not support the concept of multi waypoint planning like descartes and trajopt. Because of this
 * every plan instruction will be its a seperate ompl motion plan and therefore planning information is relevent
 * for this motion planner in the profile.
 */
class OMPLRealVectorPlanProfile : public OMPLPlanProfile
{
public:
  using Ptr = std::shared_ptr<OMPLRealVectorPlanProfile>;
  using ConstPtr = std::shared_ptr<const OMPLRealVectorPlanProfile>;

  OMPLRealVectorPlanProfile();

  /** @brief The OMPL parallel planner solver config */
  OMPLSolverConfig solver_config;

  /** @brief The collision check configuration */
  tesseract_collision::CollisionCheckConfig collision_check_config;

  std::unique_ptr<OMPLSolverConfig> createSolverConfig() const override;

  OMPLStateExtractor createStateExtractor(const tesseract_kinematics::JointGroup& manip) const override;

  std::unique_ptr<ompl::geometric::SimpleSetup>
  createSimpleSetup(const MoveInstructionPoly& start_instruction,
                    const MoveInstructionPoly& end_instruction,
                    const tesseract_common::ManipulatorInfo& composite_mi,
                    const std::shared_ptr<const tesseract_environment::Environment>& env) const override;

protected:
  static void applyGoalStates(ompl::geometric::SimpleSetup& simple_setup,
                              const tesseract_kinematics::KinGroupIKInput& ik_input,
                              const tesseract_kinematics::KinematicGroup& manip,
                              tesseract_collision::DiscreteContactManager& contact_checker);

  static void applyStartStates(ompl::geometric::SimpleSetup& simple_setup,
                               const tesseract_kinematics::KinGroupIKInput& ik_input,
                               const tesseract_kinematics::KinematicGroup& manip,
                               tesseract_collision::DiscreteContactManager& contact_checker);

  static void applyGoalStates(ompl::geometric::SimpleSetup& simple_setup,
                              const Eigen::VectorXd& joint_waypoint,
                              const tesseract_kinematics::JointGroup& manip,
                              tesseract_collision::DiscreteContactManager& contact_checker);

  static void applyStartStates(ompl::geometric::SimpleSetup& simple_setup,
                               const Eigen::VectorXd& joint_waypoint,
                               const tesseract_kinematics::JointGroup& manip,
                               tesseract_collision::DiscreteContactManager& contact_checker);

  /**
   * @brief Create state sampler allocator
   * @param env The environment
   * @param prob The OMPL problem
   * @return OMPL state sampler allocator
   */
  virtual std::function<std::shared_ptr<ompl::base::StateSampler>(const ompl::base::StateSpace*)>
  createStateSamplerAllocator(const std::shared_ptr<const tesseract_environment::Environment>& /*env*/,
                              const std::shared_ptr<const tesseract_kinematics::JointGroup>& manip) const;

  /**
   * @brief Create state validator which should not be the collision state validator
   * @param simple_setup The OMPL Simple Setup
   * @param env The environment
   * @param manip The manipulator
   * @param state_extractor The state extractor
   * @return OMPL state validator
   */
  virtual std::unique_ptr<ompl::base::StateValidityChecker>
  createStateValidator(const ompl::geometric::SimpleSetup& simple_setup,
                       const std::shared_ptr<const tesseract_environment::Environment>& env,
                       const std::shared_ptr<const tesseract_kinematics::JointGroup>& manip,
                       const OMPLStateExtractor& state_extractor) const;

  /**
   * @brief Create collision state validator
   * @param simple_setup The OMPL Simple Setup
   * @param env The environment
   * @param manip The manipulator
   * @param state_extractor The state extractor
   * @return OMPL state collision validator
   */
  virtual std::unique_ptr<ompl::base::StateValidityChecker>
  createCollisionStateValidator(const ompl::geometric::SimpleSetup& simple_setup,
                                const std::shared_ptr<const tesseract_environment::Environment>& env,
                                const std::shared_ptr<const tesseract_kinematics::JointGroup>& manip,
                                const OMPLStateExtractor& state_extractor) const;

  /**
   * @brief Create motion validator
   * @param simple_setup The OMPL Simple Setup
   * @param env The environment
   * @param manip The manipulator
   * @param state_extractor The state extractor
   * @return OMPL motion validator
   */
  virtual std::unique_ptr<ompl::base::MotionValidator>
  createMotionValidator(const ompl::geometric::SimpleSetup& simple_setup,
                        const std::shared_ptr<const tesseract_environment::Environment>& env,
                        const std::shared_ptr<const tesseract_kinematics::JointGroup>& manip,
                        const OMPLStateExtractor& state_extractor,
                        const std::shared_ptr<ompl::base::StateValidityChecker>& svc_without_collision) const;

  /**
   * @brief Create OMPL optimization object
   * @param simple_setup The OMPL Simple Setup
   * @param env The environment
   * @param manip The manipulator
   * @param state_extractor The state extractor
   * @return OMPL optimization objective
   */
  virtual std::unique_ptr<ompl::base::OptimizationObjective>
  createOptimizationObjective(const ompl::geometric::SimpleSetup& simple_setup,
                              const std::shared_ptr<const tesseract_environment::Environment>& env,
                              const std::shared_ptr<const tesseract_kinematics::JointGroup>& manip,
                              const OMPLStateExtractor& state_extractor) const;

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};
}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::OMPLRealVectorPlanProfile)

#endif  // TESSERACT_MOTION_PLANNERS_OMPL_OMPL_REAL_VECTOR_PLAN_PROFILE_H
