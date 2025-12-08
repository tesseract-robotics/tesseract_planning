#include <tesseract_command_language/cereal_serialization.h>

// These must be include before calling macro CEREAL_REGISTER_TYPE
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

CEREAL_REGISTER_TYPE(tesseract_planning::CartesianWaypointPoly)
CEREAL_REGISTER_TYPE(tesseract_planning::JointWaypointPoly)
CEREAL_REGISTER_TYPE(tesseract_planning::StateWaypointPoly)
CEREAL_REGISTER_TYPE(tesseract_planning::MoveInstructionPoly)
CEREAL_REGISTER_TYPE(tesseract_planning::CartesianWaypoint)
CEREAL_REGISTER_TYPE(tesseract_planning::JointWaypoint)
CEREAL_REGISTER_TYPE(tesseract_planning::StateWaypoint)
CEREAL_REGISTER_TYPE(tesseract_planning::MoveInstruction)
CEREAL_REGISTER_TYPE(tesseract_planning::CompositeInstruction)
CEREAL_REGISTER_TYPE(tesseract_planning::CompositeInstructionAnyPoly)
CEREAL_REGISTER_TYPE(tesseract_planning::SetAnalogInstruction)
CEREAL_REGISTER_TYPE(tesseract_planning::SetDigitalInstruction)
CEREAL_REGISTER_TYPE(tesseract_planning::SetToolInstruction)
CEREAL_REGISTER_TYPE(tesseract_planning::TimerInstruction)
CEREAL_REGISTER_TYPE(tesseract_planning::WaitInstruction)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::WaypointInterface, tesseract_planning::CartesianWaypointPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::WaypointInterface, tesseract_planning::JointWaypointPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::WaypointInterface, tesseract_planning::StateWaypointPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::InstructionInterface, tesseract_planning::MoveInstructionPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::CartesianWaypointInterface,
                                     tesseract_planning::CartesianWaypoint)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::JointWaypointInterface, tesseract_planning::JointWaypoint)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::StateWaypointInterface, tesseract_planning::StateWaypoint)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::MoveInstructionInterface, tesseract_planning::MoveInstruction)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::InstructionInterface, tesseract_planning::CompositeInstruction)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::InstructionInterface, tesseract_planning::SetAnalogInstruction)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::InstructionInterface,
                                     tesseract_planning::SetDigitalInstruction)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::InstructionInterface, tesseract_planning::SetToolInstruction)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::InstructionInterface, tesseract_planning::TimerInstruction)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::InstructionInterface, tesseract_planning::WaitInstruction)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_planning::CompositeInstructionAnyPoly)

// LCOV_EXCL_START
CEREAL_REGISTER_DYNAMIC_INIT(tesseract_command_language_cereal)
// LCOV_EXCL_STOP
