#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_FWD_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_FWD_H

namespace tesseract_planning
{
// trajopt_collision_config.h
struct CollisionCostConfig;
struct CollisionConstraintConfig;

// trajopt_motion_planner.h
class TrajOptMotionPlanner;

// profiles
struct TrajOptTermInfos;
struct TrajOptWaypointInfo;
class TrajOptMoveProfile;
class TrajOptCompositeProfile;
class TrajOptSolverProfile;

class TrajOptOSQPSolverProfile;
class TrajOptDefaultMoveProfile;
class TrajOptDefaultCompositeProfile;
}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_FWD_H
