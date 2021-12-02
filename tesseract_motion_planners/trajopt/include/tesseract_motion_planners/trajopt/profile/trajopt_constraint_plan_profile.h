#ifndef TRAJOPT_CONSTRAINT_PLAN_PROFILE_H
#define TRAJOPT_CONSTRAINT_PLAN_PROFILE_H

#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>

#ifdef SWIG
%shared_ptr(tesseract_planning::TrajOptConstraintPlanProfile)
#endif  // SWIG

namespace tesseract_planning
{
class TrajOptConstraintPlanProfile : public TrajOptDefaultPlanProfile
{
public:
  using Ptr = std::shared_ptr<TrajOptConstraintPlanProfile>;
  using ConstPtr = std::shared_ptr<const TrajOptConstraintPlanProfile>;

  TrajOptConstraintPlanProfile() = default;
  ~TrajOptConstraintPlanProfile() override = default;

  void apply(trajopt::ProblemConstructionInfo& pci,
             const CartesianWaypoint& cartesian_waypoint,
             const Instruction& parent_instruction,
             const ManipulatorInfo& manip_info,
             const std::vector<std::string>& active_links,
             int index) const override;
};
}  // namespace tesseract_planning

#endif