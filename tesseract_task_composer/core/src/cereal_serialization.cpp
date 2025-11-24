#include <tesseract_task_composer/core/cereal_serialization.h>

// These must be include before calling macro CEREAL_REGISTER_TYPE
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

CEREAL_REGISTER_TYPE(tesseract_planning::TaskComposerDataStoragePtrAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface,
                                     tesseract_planning::TaskComposerDataStoragePtrAnyPoly)

CEREAL_REGISTER_DYNAMIC_INIT(tesseract_task_composer_core_cereal)
