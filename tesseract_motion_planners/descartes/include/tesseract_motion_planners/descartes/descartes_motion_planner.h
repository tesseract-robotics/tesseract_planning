#ifndef TESSERACT_MOTION_PLANNERS_DECARTES_MOTION_PLANNER_H
#define TESSERACT_MOTION_PLANNERS_DECARTES_MOTION_PLANNER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <descartes_light/core/edge_evaluator.h>
#include <descartes_light/core/waypoint_sampler.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/core/planner.h>
#include <tesseract_motion_planners/descartes/descartes_problem.h>
#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>

namespace tesseract_planning
{
template <typename FloatType>
class DescartesMotionPlanner : public MotionPlanner
{
public:
  /** @brief Construct a basic planner */
  DescartesMotionPlanner(std::string name);
  ~DescartesMotionPlanner() override = default;
  DescartesMotionPlanner(const DescartesMotionPlanner&) = delete;
  DescartesMotionPlanner& operator=(const DescartesMotionPlanner&) = delete;
  DescartesMotionPlanner(DescartesMotionPlanner&&) noexcept = delete;
  DescartesMotionPlanner& operator=(DescartesMotionPlanner&&) noexcept = delete;

  PlannerResponse solve(const PlannerRequest& request) const override;

  bool terminate() override;

  void clear() override;

  MotionPlanner::Ptr clone() const override;

  virtual std::shared_ptr<DescartesProblem<FloatType>> createProblem(const PlannerRequest& request) const;
};

using DescartesMotionPlannerD = DescartesMotionPlanner<double>;
using DescartesMotionPlannerF = DescartesMotionPlanner<float>;

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_DECARTES_MOTION_PLANNER_H
