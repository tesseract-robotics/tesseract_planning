#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_FWD_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_FWD_H

namespace tesseract_planning
{
// trajopt_collision_config.h
struct CollisionCostConfig;
struct CollisionConstraintConfig;

// trajopt_motion_planner.h
class TrajOptMotionPlanner;

// trajopt_waypoint_config.h
struct CartesianWaypointConfig;
struct JointWaypointConfig;

// profiles
class TrajOptPlanProfile;
class TrajOptCompositeProfile;
class TrajOptSolverProfile;

class TrajOptOSQPSolverProfile;
class TrajOptDefaultPlanProfile;
class TrajOptDefaultCompositeProfile;
}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_FWD_H
