#ifndef TESSERACT_COMMAND_LANGUAGE_FWD_H
#define TESSERACT_COMMAND_LANGUAGE_FWD_H

namespace tesseract_planning
{
// Waypoint Poly
struct WaypointPoly;
struct CartesianWaypointPoly;
struct JointWaypointPoly;
struct StateWaypointPoly;

// Waypoints
class CartesianWaypoint;
class JointWaypoint;
class StateWaypoint;

// Instruction Poly
struct InstructionPoly;
enum class MoveInstructionType : int;
struct MoveInstructionPoly;

// Instructions
enum class CompositeInstructionOrder;
class CompositeInstruction;
class MoveInstruction;
class SetAnalogInstruction;
class SetToolInstruction;
enum class TimerInstructionType : int;
class TimerInstruction;
enum class WaitInstructionType : int;
class WaitInstruction;

// Profile Dictionary
class Profile;
class ProfileDictionary;
}  // namespace tesseract_planning
#endif  // TESSERACT_COMMAND_LANGUAGE_FWD_H
