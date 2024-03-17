#ifndef TESSERACT_MOTION_PLANNERS_SIMPLE_FWD_H
#define TESSERACT_MOTION_PLANNERS_SIMPLE_FWD_H

namespace tesseract_planning
{
// interpolation.h
struct JointGroupInstructionInfo;
struct KinematicGroupInstructionInfo;

// simple_motion_planner.h
class SimpleMotionPlanner;

// profiles
class SimplePlannerPlanProfile;
class SimplePlannerCompositeProfile;
class SimplePlannerLVSPlanProfile;
class SimplePlannerLVSNoIKPlanProfile;
class SimplePlannerFixedSizePlanProfile;
class SimplePlannerFixedSizeAssignPlanProfile;
}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_SIMPLE_FWD_H
