#ifndef TESSERACT_COMMAND_LANGUAGE_FWD_H
#define TESSERACT_COMMAND_LANGUAGE_FWD_H

#include <cstdint>

namespace tesseract_planning
{
// Waypoint Poly
class WaypointPoly;
class CartesianWaypointPoly;
class JointWaypointPoly;
class StateWaypointPoly;

// Waypoints
class CartesianWaypoint;
class JointWaypoint;
class StateWaypoint;

// Instruction Poly
class InstructionPoly;
enum class MoveInstructionType : std::uint8_t;
class MoveInstructionPoly;

// Instructions
enum class CompositeInstructionOrder : std::uint8_t;
class CompositeInstruction;
class MoveInstruction;
class SetAnalogInstruction;
class SetToolInstruction;
enum class TimerInstructionType : std::uint8_t;
class TimerInstruction;
enum class WaitInstructionType : std::uint8_t;
class WaitInstruction;

// Profile Dictionary
class Profile;
class ProfileDictionary;
}  // namespace tesseract_planning
#endif  // TESSERACT_COMMAND_LANGUAGE_FWD_H
