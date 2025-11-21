#ifndef TESSERACT_TASK_COMPOSER_CEREAL_SERIALIZATION_H
#define TESSERACT_TASK_COMPOSER_CEREAL_SERIALIZATION_H

#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_keys.h>
#include <tesseract_task_composer/core/task_composer_log.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>
#include <tesseract_task_composer/core/task_composer_node_ports.h>

#include <tesseract_common/cereal_serialization.h>

#include <cereal/cereal.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/variant.hpp>
#include <cereal/types/chrono.hpp>
#include <cereal/types/polymorphic.hpp>

#include <mutex>

namespace tesseract_planning
{
template <class Archive>
void serialize(Archive& ar, TaskComposerKeys& obj)
{
  ar(cereal::make_nvp("keys", obj.keys_));
}

template <class Archive>
void serialize(Archive& ar, TaskComposerNodePorts& obj)
{
  ar(cereal::make_nvp("input_required", obj.input_required));
  ar(cereal::make_nvp("input_optional", obj.input_optional));
  ar(cereal::make_nvp("output_required", obj.output_required));
  ar(cereal::make_nvp("output_optional", obj.output_optional));
}

template <class Archive>
void serialize(Archive& ar, TaskComposerDataStorage& obj)
{
  std::unique_lock lock(obj.mutex_);
  ar(cereal::make_nvp("name", obj.name_));
  ar(cereal::make_nvp("data", obj.data_));
}

template <class Archive>
void serialize(Archive& ar, TaskComposerNodeInfo& obj)
{
  ar(cereal::make_nvp("name", obj.name));
  ar(cereal::make_nvp("ns", obj.ns));
  ar(cereal::make_nvp("uuid", obj.uuid));
  ar(cereal::make_nvp("root_uuid", obj.root_uuid));
  ar(cereal::make_nvp("parent_uuid", obj.parent_uuid));
  ar(cereal::make_nvp("type", obj.type));
  ar(cereal::make_nvp("type_hash_code", obj.type_hash_code));
  ar(cereal::make_nvp("conditional", obj.conditional));
  ar(cereal::make_nvp("return_value", obj.return_value));
  ar(cereal::make_nvp("status_code", obj.status_code));
  ar(cereal::make_nvp("status_message", obj.status_message));
  ar(cereal::make_nvp("start_time", obj.start_time));
  ar(cereal::make_nvp("elapsed_time", obj.elapsed_time));
  ar(cereal::make_nvp("inbound_edges", obj.inbound_edges));
  ar(cereal::make_nvp("outbound_edges", obj.outbound_edges));
  ar(cereal::make_nvp("input_keys", obj.input_keys));
  ar(cereal::make_nvp("terminals", obj.terminals));
  ar(cereal::make_nvp("triggers_abort", obj.triggers_abort));
  ar(cereal::make_nvp("color", obj.color));
  ar(cereal::make_nvp("dotgraph", obj.dotgraph));
  ar(cereal::make_nvp("data_storage", obj.data_storage));
  ar(cereal::make_nvp("aborted", obj.aborted));
}

template <class Archive>
void serialize(Archive& ar, TaskComposerNodeInfoContainer& obj)
{
  std::unique_lock<std::shared_mutex> lock(obj.mutex_);
  ar(cereal::make_nvp("root_node", obj.root_node_));
  ar(cereal::make_nvp("aborting_node", obj.aborting_node_));
  ar(cereal::make_nvp("info_map", obj.info_map_));
}

template <class Archive>
void serialize(Archive& ar, TaskComposerContext& obj)
{
  ar(cereal::make_nvp("name", obj.name));
  ar(cereal::make_nvp("dotgraph", obj.dotgraph));
  ar(cereal::make_nvp("data_storage", obj.data_storage));
  ar(cereal::make_nvp("task_infos", obj.task_infos));
  ar(cereal::make_nvp("aborted", obj.aborted_));
}

template <class Archive>
void serialize(Archive& ar, TaskComposerLog& obj)
{
  ar(cereal::make_nvp("description", obj.description));
  ar(cereal::make_nvp("description", obj.description));
  ar(cereal::make_nvp("context", obj.context));
  ar(cereal::make_nvp("dotgraph", obj.dotgraph));
}

}  // namespace tesseract_planning

// These must be include before calling macro CEREAL_REGISTER_TYPE
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

CEREAL_REGISTER_TYPE(tesseract_planning::TaskComposerDataStoragePtrAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface,
                                     tesseract_planning::TaskComposerDataStoragePtrAnyPoly)

#endif  // TESSERACT_TASK_COMPOSER_CEREAL_SERIALIZATION_H
