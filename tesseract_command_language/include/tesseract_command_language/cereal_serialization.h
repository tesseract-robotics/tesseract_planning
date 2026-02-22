#ifndef TESSERACT_COMMAND_LANGUAGE_CEREAL_SERIALIZATION_H
#define TESSERACT_COMMAND_LANGUAGE_CEREAL_SERIALIZATION_H

#include <tesseract_command_language/poly/waypoint_poly.h>
#include <tesseract_command_language/poly/cartesian_waypoint_poly.h>
#include <tesseract_command_language/poly/joint_waypoint_poly.h>
#include <tesseract_command_language/poly/state_waypoint_poly.h>
#include <tesseract_command_language/poly/instruction_poly.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/set_analog_instruction.h>
#include <tesseract_command_language/set_digital_instruction.h>
#include <tesseract_command_language/set_tool_instruction.h>
#include <tesseract_command_language/timer_instruction.h>
#include <tesseract_command_language/wait_instruction.h>

#include <tesseract_common/cereal_serialization.h>

#include <cereal/cereal.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/polymorphic.hpp>

namespace tesseract::command_language
{
template <class Archive>
void serialize(Archive& /*ar*/, WaypointInterface& /*obj*/)
{
}

template <class Archive>
void serialize(Archive& ar, WaypointPoly& obj)
{
  ar(cereal::make_nvp("impl", obj.impl_));
}

template <class Archive>
void serialize(Archive& /*ar*/, CartesianWaypointInterface& /*obj*/)
{
}

template <class Archive>
void serialize(Archive& ar, CartesianWaypointPoly& obj)
{
  ar(cereal::base_class<WaypointInterface>(&obj));
  ar(cereal::make_nvp("impl", obj.impl_));
}

template <class Archive>
void serialize(Archive& /*ar*/, JointWaypointInterface& /*obj*/)
{
}

template <class Archive>
void serialize(Archive& ar, JointWaypointPoly& obj)
{
  ar(cereal::base_class<WaypointInterface>(&obj));
  ar(cereal::make_nvp("impl", obj.impl_));
}

template <class Archive>
void serialize(Archive& /*ar*/, StateWaypointInterface& /*obj*/)
{
}

template <class Archive>
void serialize(Archive& ar, StateWaypointPoly& obj)
{
  ar(cereal::base_class<WaypointInterface>(&obj));
  ar(cereal::make_nvp("impl", obj.impl_));
}

template <class Archive>
void serialize(Archive& /*ar*/, InstructionInterface& /*obj*/)
{
}

template <class Archive>
void serialize(Archive& ar, InstructionPoly& obj)
{
  ar(cereal::make_nvp("impl", obj.impl_));
}

template <class Archive>
void serialize(Archive& /*ar*/, MoveInstructionInterface& /*obj*/)
{
}

template <class Archive>
void serialize(Archive& ar, MoveInstructionPoly& obj)
{
  ar(cereal::base_class<InstructionInterface>(&obj));
  ar(cereal::make_nvp("impl", obj.impl_));
}

template <class Archive>
void serialize(Archive& ar, CartesianWaypoint& obj)
{
  ar(cereal::base_class<CartesianWaypointInterface>(&obj));
  ar(cereal::make_nvp("name", obj.name_));
  ar(cereal::make_nvp("transform", obj.transform_));
  ar(cereal::make_nvp("upper_tolerance", obj.upper_tolerance_));
  ar(cereal::make_nvp("lower_tolerance", obj.lower_tolerance_));
  ar(cereal::make_nvp("seed", obj.seed_));
}

template <class Archive>
void serialize(Archive& ar, JointWaypoint& obj)
{
  ar(cereal::base_class<JointWaypointInterface>(&obj));
  ar(cereal::make_nvp("name", obj.name_));
  ar(cereal::make_nvp("names", obj.names_));
  ar(cereal::make_nvp("position", obj.position_));
  ar(cereal::make_nvp("upper_tolerance", obj.upper_tolerance_));
  ar(cereal::make_nvp("lower_tolerance", obj.lower_tolerance_));
  ar(cereal::make_nvp("is_constrained", obj.is_constrained_));
}

template <class Archive>
void serialize(Archive& ar, StateWaypoint& obj)
{
  ar(cereal::base_class<StateWaypointInterface>(&obj));
  ar(cereal::make_nvp("name", obj.name_));
  ar(cereal::make_nvp("joint_names", obj.joint_names_));
  ar(cereal::make_nvp("position", obj.position_));
  ar(cereal::make_nvp("velocity", obj.velocity_));
  ar(cereal::make_nvp("acceleration", obj.acceleration_));
  ar(cereal::make_nvp("effort", obj.effort_));
  ar(cereal::make_nvp("time", obj.time_));
}

template <class Archive>
void serialize(Archive& ar, MoveInstruction& obj)
{
  ar(cereal::base_class<MoveInstructionInterface>(&obj));
  ar(cereal::make_nvp("uuid", obj.uuid_));
  ar(cereal::make_nvp("parent_uuid", obj.parent_uuid_));
  ar(cereal::make_nvp("move_type", obj.move_type_));
  ar(cereal::make_nvp("description", obj.description_));
  ar(cereal::make_nvp("profile", obj.profile_));
  ar(cereal::make_nvp("path_profile", obj.path_profile_));
  ar(cereal::make_nvp("profile_overrides", obj.profile_overrides_));
  ar(cereal::make_nvp("waypoint", obj.waypoint_));
  ar(cereal::make_nvp("manipulator_info", obj.manipulator_info_));
}

template <class Archive>
void serialize(Archive& ar, CompositeInstruction& obj)
{
  ar(cereal::base_class<InstructionInterface>(&obj));
  ar(cereal::make_nvp("uuid", obj.uuid_));
  ar(cereal::make_nvp("parent_uuid", obj.parent_uuid_));
  ar(cereal::make_nvp("description", obj.description_));
  ar(cereal::make_nvp("manipulator_info", obj.manipulator_info_));
  ar(cereal::make_nvp("profile", obj.profile_));
  ar(cereal::make_nvp("profile_overrides", obj.profile_overrides_));
  ar(cereal::make_nvp("order", obj.order_));
  ar(cereal::make_nvp("user_data", obj.user_data_));
  ar(cereal::make_nvp("container", obj.container_));
}

template <class Archive>
void serialize(Archive& ar, SetAnalogInstruction& obj)
{
  ar(cereal::base_class<InstructionInterface>(&obj));
  ar(cereal::make_nvp("uuid", obj.uuid_));
  ar(cereal::make_nvp("parent_uuid", obj.parent_uuid_));
  ar(cereal::make_nvp("description", obj.description_));
  ar(cereal::make_nvp("key", obj.key_));
  ar(cereal::make_nvp("index", obj.index_));
  ar(cereal::make_nvp("value", obj.value_));
}

template <class Archive>
void serialize(Archive& ar, SetDigitalInstruction& obj)
{
  ar(cereal::base_class<InstructionInterface>(&obj));
  ar(cereal::make_nvp("uuid", obj.uuid_));
  ar(cereal::make_nvp("parent_uuid", obj.parent_uuid_));
  ar(cereal::make_nvp("description", obj.description_));
  ar(cereal::make_nvp("key", obj.key_));
  ar(cereal::make_nvp("index", obj.index_));
  ar(cereal::make_nvp("value", obj.value_));
}

template <class Archive>
void serialize(Archive& ar, SetToolInstruction& obj)
{
  ar(cereal::base_class<InstructionInterface>(&obj));
  ar(cereal::make_nvp("uuid", obj.uuid_));
  ar(cereal::make_nvp("parent_uuid", obj.parent_uuid_));
  ar(cereal::make_nvp("description", obj.description_));
  ar(cereal::make_nvp("tool_id", obj.tool_id_));
}

template <class Archive>
void serialize(Archive& ar, TimerInstruction& obj)
{
  ar(cereal::base_class<InstructionInterface>(&obj));
  ar(cereal::make_nvp("uuid", obj.uuid_));
  ar(cereal::make_nvp("parent_uuid", obj.parent_uuid_));
  ar(cereal::make_nvp("description", obj.description_));
  ar(cereal::make_nvp("timer_type", obj.timer_type_));
  ar(cereal::make_nvp("timer_time", obj.timer_time_));
  ar(cereal::make_nvp("timer_io", obj.timer_io_));
}

template <class Archive>
void serialize(Archive& ar, WaitInstruction& obj)
{
  ar(cereal::base_class<InstructionInterface>(&obj));
  ar(cereal::make_nvp("uuid", obj.uuid_));
  ar(cereal::make_nvp("parent_uuid", obj.parent_uuid_));
  ar(cereal::make_nvp("description", obj.description_));
  ar(cereal::make_nvp("wait_type", obj.wait_type_));
  ar(cereal::make_nvp("wait_time", obj.wait_time_));
  ar(cereal::make_nvp("wait_io", obj.wait_io_));
}

}  // namespace tesseract::command_language

#endif  // TESSERACT_COMMAND_LANGUAGE_CEREAL_SERIALIZATION_H
