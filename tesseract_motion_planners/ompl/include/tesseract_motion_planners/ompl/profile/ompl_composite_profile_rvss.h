#ifndef TESSERACT_MOTION_PLANNERS_OMPL_PROFILE_OMPL_COMPOSITE_PROFILE_RVSS_H
#define TESSERACT_MOTION_PLANNERS_OMPL_PROFILE_OMPL_COMPOSITE_PROFILE_RVSS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/StateSampler.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/common.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>

namespace tesseract_planning
{
using StateValidityCheckerAllocator =
    std::function<ompl::base::StateValidityCheckerPtr(const ompl::base::SpaceInformationPtr&)>;

using MotionValidatorAllocator = std::function<ompl::base::MotionValidatorPtr(const ompl::base::SpaceInformationPtr&)>;

using OptimizationObjectiveAllocator =
    std::function<ompl::base::OptimizationObjectivePtr(const ompl::base::SpaceInformationPtr&)>;

/**
 * @brief OMPL composite profile for real-vector state spaces
 */
class OMPLCompositeProfileRVSS : public CompositeProfile
{
public:
  /**
   * @brief The collision check configuration
   * @details Different validators will be created depending on the type of collision checking requested
   *   - Discrete:
   *     - Discrete collision state validator, discrete motion validator
   *   - Continuous:
   *     - Continuous motion validator
   *   - None:
   *     - No additional validators will be created
   */
  tesseract_collision::CollisionCheckConfig collision_check_config;

  /** @brief The state sampler allocator. This can be null and it will use Tesseract default state sampler allocator. */
  ompl::base::StateSamplerAllocator state_sampler_allocator{ nullptr };

  /**
   * @brief The ompl state validity checker
   * @details If nullptr and collision checking enabled it uses StateCollisionValidator
   */
  StateValidityCheckerAllocator state_validator_allocator{ nullptr };

  /**
   * @brief The ompl motion validator.
   * @details If nullptr and continuous collision checking enabled it used ContinuousMotionValidator
   */
  MotionValidatorAllocator motion_validator_allocator{ nullptr };

  /**
   * @brief Function for allocating the optimization objective
   * @details Default is the minimization of path length
   */
  OptimizationObjectiveAllocator optimization_objective_allocator = [](const ompl::base::SpaceInformationPtr& si) {
    return std::make_shared<ompl::base::PathLengthOptimizationObjective>(si);
  };

  /**
   * @brief Creates the OMPL composite profile data given a composite instruction and environment
   */
  std::any create(const CompositeInstruction& instruction,
                  const tesseract_environment::Environment& env) const override;

  /**
   * @brief Seed for OMPL random number generator
   * @details When this value is greater than or equal to zero, the OMPL random number generator will produce a
   * determinstic sequence of random samples
   */
  long rng_seed = -1;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int)
  {
  }
};

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_OMPL_PROFILE_OMPL_COMPOSITE_PROFILE_RVSS_H
