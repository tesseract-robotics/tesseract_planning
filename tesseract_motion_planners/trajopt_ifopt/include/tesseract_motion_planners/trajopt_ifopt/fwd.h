#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_FWD_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_FWD_H

namespace tesseract_planning
{
// trajopt_ifopt_motion_planner.h
class TrajOptIfoptMotionPlanner;

// trajopt_ifopt_problem.h
enum class TrajOptIfoptTermType;
struct TrajOptIfoptProblem;

// profiles
class TrajOptIfoptPlanProfile;
class TrajOptIfoptCompositeProfile;
class TrajOptIfoptSolverProfile;

class TrajOptIfoptDefaultSolverProfile;
class TrajOptIfoptDefaultPlanProfile;
class TrajOptIfoptDefaultCompositeProfile;
}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_FWD_H
