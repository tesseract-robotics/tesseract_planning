#include <tesseract_task_composer/core/cereal_serialization.h>

// These must be include before calling macro CEREAL_REGISTER_TYPE
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

CEREAL_REGISTER_TYPE(tesseract::task_composer::TaskComposerDataStoragePtrAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::AnyInterface,
                                     tesseract::task_composer::TaskComposerDataStoragePtrAnyPoly)

// LCOV_EXCL_START
CEREAL_REGISTER_DYNAMIC_INIT(tesseract_task_composer_core_cereal)
// LCOV_EXCL_STOP
