#ifndef TESSERACT_PLANNING_OMPL_PROFILE_OMPL_WAYPOINT_PROFILE_H
#define TESSERACT_PLANNING_OMPL_PROFILE_OMPL_WAYPOINT_PROFILE_H

#include <tesseract_motion_planners/core/planner.h>

namespace tesseract_planning
{
struct OMPLWaypointProfile : public WaypointProfile<std::vector<Eigen::VectorXd>>
{
  virtual std::vector<Eigen::VectorXd> create(const Instruction& instruction,
                                              tesseract_environment::Environment::ConstPtr env) const override;
};

} // namespace tesseract_planning

#endif // TESSERACT_PLANNING_OMPL_PROFILE_OMPL_WAYPOINT_PROFILE_H
