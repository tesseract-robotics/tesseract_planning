#ifndef TESSERACT_MOTION_PLANNERS_DECARTES_MOTION_PLANNER_H
#define TESSERACT_MOTION_PLANNERS_DECARTES_MOTION_PLANNER_H

#include <tesseract_motion_planners/core/planner.h>

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

  std::unique_ptr<MotionPlanner> clone() const override;
};

using DescartesMotionPlannerD = DescartesMotionPlanner<double>;
using DescartesMotionPlannerF = DescartesMotionPlanner<float>;

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_DECARTES_MOTION_PLANNER_H
