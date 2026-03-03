#ifndef TESSERACT_MOTION_PLANNERS_SIMPLE_FWD_H
#define TESSERACT_MOTION_PLANNERS_SIMPLE_FWD_H

namespace tesseract::motion_planners
{
// interpolation.h
struct JointGroupInstructionInfo;
struct KinematicGroupInstructionInfo;

// simple_motion_planner.h
class SimpleMotionPlanner;

// profiles
class SimplePlannerMoveProfile;
class SimplePlannerCompositeProfile;
class SimplePlannerLVSMoveProfile;
class SimplePlannerLVSNoIKMoveProfile;
class SimplePlannerFixedSizeMoveProfile;
class SimplePlannerFixedSizeAssignMoveProfile;
}  // namespace tesseract::motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_SIMPLE_FWD_H
