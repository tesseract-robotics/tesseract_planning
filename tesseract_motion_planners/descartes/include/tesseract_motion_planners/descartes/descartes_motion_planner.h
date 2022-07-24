#ifndef TESSERACT_MOTION_PLANNERS_DECARTES_MOTION_PLANNER_H
#define TESSERACT_MOTION_PLANNERS_DECARTES_MOTION_PLANNER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <descartes_light/core/edge_evaluator.h>
#include <descartes_light/core/waypoint_sampler.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/default_planner_namespaces.h>
#include <tesseract_motion_planners/core/planner.h>
#include <tesseract_motion_planners/descartes/descartes_motion_planner_status_category.h>
#include <tesseract_motion_planners/descartes/descartes_problem.h>
#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>

namespace tesseract_planning
{
template <typename FloatType>
class DescartesMotionPlanner : public MotionPlanner
{
public:
  /** @brief Construct a basic planner */
  DescartesMotionPlanner(std::string name = profile_ns::DESCARTES_DEFAULT_NAMESPACE);
  ~DescartesMotionPlanner() override = default;
  DescartesMotionPlanner(const DescartesMotionPlanner&) = delete;
  DescartesMotionPlanner& operator=(const DescartesMotionPlanner&) = delete;
  DescartesMotionPlanner(DescartesMotionPlanner&&) noexcept = delete;
  DescartesMotionPlanner& operator=(DescartesMotionPlanner&&) noexcept = delete;

  const std::string& getName() const override;

  /**
   * @brief Sets up the optimizer and solves a SQP problem read from json with no callbacks and default parameters
   * @param response The results of the optimization. Primary output is the optimized joint trajectory
   * @param check_type The type of validation check to be performed on the planned trajectory
   * @param verbose Boolean indicating whether logging information about the motion planning solution should be printed
   * to console
   * @return true if optimization complete
   */
  tesseract_common::StatusCode solve(const PlannerRequest& request,
                                     PlannerResponse& response,
                                     bool verbose = false) const override;

  bool checkUserInput(const PlannerRequest& request);

  bool terminate() override;

  void clear() override;

  MotionPlanner::Ptr clone() const override;

  virtual std::shared_ptr<DescartesProblem<FloatType>> createProblem(const PlannerRequest& request) const;

private:
  /** @brief The planners status codes */
  std::string name_;
  std::shared_ptr<const DescartesMotionPlannerStatusCategory> status_category_;
};

using DescartesMotionPlannerD = DescartesMotionPlanner<double>;
using DescartesMotionPlannerF = DescartesMotionPlanner<float>;

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_DECARTES_MOTION_PLANNER_H
