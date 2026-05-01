#ifndef TESSERACT_TASK_COMPOSER_PLANNING_CEREAL_SERIALIZATION_IMPL_HPP
#define TESSERACT_TASK_COMPOSER_PLANNING_CEREAL_SERIALIZATION_IMPL_HPP

// These must be include before calling macro CEREAL_REGISTER_TYPE
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

CEREAL_REGISTER_TYPE(tesseract::task_composer::ContactCheckProfile)
CEREAL_REGISTER_TYPE(tesseract::task_composer::FixStateBoundsProfile)
CEREAL_REGISTER_TYPE(tesseract::task_composer::FixStateCollisionProfile)
CEREAL_REGISTER_TYPE(tesseract::task_composer::KinematicLimitsCheckProfile)
CEREAL_REGISTER_TYPE(tesseract::task_composer::MinLengthProfile)
CEREAL_REGISTER_TYPE(tesseract::task_composer::ProfileSwitchProfile)
CEREAL_REGISTER_TYPE(tesseract::task_composer::UpsampleTrajectoryProfile)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile, tesseract::task_composer::ContactCheckProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile, tesseract::task_composer::FixStateBoundsProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile, tesseract::task_composer::FixStateCollisionProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile, tesseract::task_composer::KinematicLimitsCheckProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile, tesseract::task_composer::MinLengthProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile, tesseract::task_composer::ProfileSwitchProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile, tesseract::task_composer::UpsampleTrajectoryProfile)

#endif  // TESSERACT_TASK_COMPOSER_PLANNING_CEREAL_SERIALIZATION_IMPL_HPP