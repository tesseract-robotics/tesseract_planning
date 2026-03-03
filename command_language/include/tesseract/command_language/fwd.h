#ifndef TESSERACT_COMMAND_LANGUAGE_FWD_H
#define TESSERACT_COMMAND_LANGUAGE_FWD_H

#include <cstdint>

namespace tesseract::command_language
{
// Waypoint Poly
class WaypointInterface;
class CartesianWaypointInterface;
class JointWaypointInterface;
class StateWaypointInterface;
class WaypointPoly;
class CartesianWaypointPoly;
class JointWaypointPoly;
class StateWaypointPoly;

// Waypoints
class CartesianWaypoint;
class JointWaypoint;
class StateWaypoint;

// Instruction Poly
class InstructionInterface;
class InstructionPoly;
enum class MoveInstructionType : std::uint8_t;
class MoveInstructionInterface;
class MoveInstructionPoly;

// Instructions
enum class CompositeInstructionOrder : std::uint8_t;
class CompositeInstruction;
class MoveInstruction;
class SetAnalogInstruction;
class SetDigitalInstruction;
class SetToolInstruction;
enum class TimerInstructionType : std::uint8_t;
class TimerInstruction;
enum class WaitInstructionType : std::uint8_t;
class WaitInstruction;

}  // namespace tesseract::command_language
#endif  // TESSERACT_COMMAND_LANGUAGE_FWD_H
