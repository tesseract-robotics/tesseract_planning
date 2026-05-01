#ifndef TESSERACT_TASK_COMPOSER_CEREAL_SERIALIZATION_IMPL_HPP
#define TESSERACT_TASK_COMPOSER_CEREAL_SERIALIZATION_IMPL_HPP

// These must be include before calling macro CEREAL_REGISTER_TYPE
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

CEREAL_REGISTER_TYPE(tesseract::task_composer::TaskComposerDataStoragePtrAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::AnyInterface,
                                     tesseract::task_composer::TaskComposerDataStoragePtrAnyPoly)

#endif  // TESSERACT_TASK_COMPOSER_CEREAL_SERIALIZATION_IMPL_HPP
