#ifndef TESSERACT_PLANNING_OMPL_PROFILE_OMPL_WAYPOINT_PROFILE_H
#define TESSERACT_PLANNING_OMPL_PROFILE_OMPL_WAYPOINT_PROFILE_H

#include <tesseract_motion_planners/core/planner.h>

namespace tesseract_planning
{
class OMPLWaypointProfile : public WaypointProfile
{
public:
  std::any create(const Instruction& instruction, const tesseract_environment::Environment& env) const override;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int)
  {
  }
};

}  // namespace tesseract_planning

#endif  // TESSERACT_PLANNING_OMPL_PROFILE_OMPL_WAYPOINT_PROFILE_H
