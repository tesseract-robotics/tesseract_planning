#include <tesseract_motion_planners/trajopt_ifopt/cereal_serialization.h>

// These must be include before calling macro CEREAL_REGISTER_TYPE
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

CEREAL_REGISTER_TYPE(tesseract_planning::TrajOptIfoptMoveProfile)
CEREAL_REGISTER_TYPE(tesseract_planning::TrajOptIfoptCompositeProfile)
CEREAL_REGISTER_TYPE(tesseract_planning::TrajOptIfoptSolverProfile)
CEREAL_REGISTER_TYPE(tesseract_planning::TrajOptIfoptDefaultMoveProfile)
CEREAL_REGISTER_TYPE(tesseract_planning::TrajOptIfoptDefaultCompositeProfile)
CEREAL_REGISTER_TYPE(tesseract_planning::TrajOptIfoptOSQPSolverProfile)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile, tesseract_planning::TrajOptIfoptMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile, tesseract_planning::TrajOptIfoptCompositeProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile, tesseract_planning::TrajOptIfoptSolverProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile, tesseract_planning::TrajOptIfoptDefaultMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile, tesseract_planning::TrajOptIfoptDefaultCompositeProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile, tesseract_planning::TrajOptIfoptOSQPSolverProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::TrajOptIfoptMoveProfile,
                                     tesseract_planning::TrajOptIfoptDefaultMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::TrajOptIfoptCompositeProfile,
                                     tesseract_planning::TrajOptIfoptDefaultCompositeProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::TrajOptIfoptSolverProfile,
                                     tesseract_planning::TrajOptIfoptOSQPSolverProfile)

// LCOV_EXCL_START
CEREAL_REGISTER_DYNAMIC_INIT(tesseract_motion_planners_trajopt_ifopt_cereal)
// LCOV_EXCL_STOP
