#include <tesseract_task_composer/planning/cereal_serialization.h>

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

// LCOV_EXCL_START
CEREAL_REGISTER_DYNAMIC_INIT(tesseract_task_composer_planning_cereal)
// LCOV_EXCL_STOP
