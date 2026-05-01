#ifndef TESSERACT_COMMAND_LANGUAGE_CEREAL_SERIALIZATION_IMPL_HPP
#define TESSERACT_COMMAND_LANGUAGE_CEREAL_SERIALIZATION_IMPL_HPP

// These must be include before calling macro CEREAL_REGISTER_TYPE
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

CEREAL_REGISTER_TYPE(tesseract::command_language::CartesianWaypointPoly)
CEREAL_REGISTER_TYPE(tesseract::command_language::JointWaypointPoly)
CEREAL_REGISTER_TYPE(tesseract::command_language::StateWaypointPoly)
CEREAL_REGISTER_TYPE(tesseract::command_language::MoveInstructionPoly)
CEREAL_REGISTER_TYPE(tesseract::command_language::CartesianWaypoint)
CEREAL_REGISTER_TYPE(tesseract::command_language::JointWaypoint)
CEREAL_REGISTER_TYPE(tesseract::command_language::StateWaypoint)
CEREAL_REGISTER_TYPE(tesseract::command_language::MoveInstruction)
CEREAL_REGISTER_TYPE(tesseract::command_language::CompositeInstruction)
CEREAL_REGISTER_TYPE(tesseract::command_language::CompositeInstructionAnyPoly)
CEREAL_REGISTER_TYPE(tesseract::command_language::SetAnalogInstruction)
CEREAL_REGISTER_TYPE(tesseract::command_language::SetDigitalInstruction)
CEREAL_REGISTER_TYPE(tesseract::command_language::SetToolInstruction)
CEREAL_REGISTER_TYPE(tesseract::command_language::TimerInstruction)
CEREAL_REGISTER_TYPE(tesseract::command_language::WaitInstruction)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::command_language::WaypointInterface,
                                     tesseract::command_language::CartesianWaypointPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::command_language::WaypointInterface,
                                     tesseract::command_language::JointWaypointPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::command_language::WaypointInterface,
                                     tesseract::command_language::StateWaypointPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::command_language::InstructionInterface,
                                     tesseract::command_language::MoveInstructionPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::command_language::CartesianWaypointInterface,
                                     tesseract::command_language::CartesianWaypoint)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::command_language::JointWaypointInterface,
                                     tesseract::command_language::JointWaypoint)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::command_language::StateWaypointInterface,
                                     tesseract::command_language::StateWaypoint)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::command_language::MoveInstructionInterface,
                                     tesseract::command_language::MoveInstruction)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::command_language::InstructionInterface,
                                     tesseract::command_language::CompositeInstruction)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::command_language::InstructionInterface,
                                     tesseract::command_language::SetAnalogInstruction)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::command_language::InstructionInterface,
                                     tesseract::command_language::SetDigitalInstruction)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::command_language::InstructionInterface,
                                     tesseract::command_language::SetToolInstruction)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::command_language::InstructionInterface,
                                     tesseract::command_language::TimerInstruction)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::command_language::InstructionInterface,
                                     tesseract::command_language::WaitInstruction)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::AnyInterface,
                                     tesseract::command_language::CompositeInstructionAnyPoly)

#endif  // TESSERACT_COMMAND_LANGUAGE_CEREAL_SERIALIZATION_IMPL_HPP
