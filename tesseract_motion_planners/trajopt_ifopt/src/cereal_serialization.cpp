#include <tesseract_motion_planners/trajopt_ifopt/cereal_serialization.h>

// These must be include before calling macro CEREAL_REGISTER_TYPE
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

CEREAL_REGISTER_TYPE(tesseract::motion_planners::TrajOptIfoptMoveProfile)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::TrajOptIfoptCompositeProfile)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::TrajOptIfoptSolverProfile)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::TrajOptIfoptDefaultMoveProfile)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::TrajOptIfoptDefaultCompositeProfile)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::TrajOptIfoptOSQPSolverProfile)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile, tesseract::motion_planners::TrajOptIfoptMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile,
                                     tesseract::motion_planners::TrajOptIfoptCompositeProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile, tesseract::motion_planners::TrajOptIfoptSolverProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile,
                                     tesseract::motion_planners::TrajOptIfoptDefaultMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile,
                                     tesseract::motion_planners::TrajOptIfoptDefaultCompositeProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile,
                                     tesseract::motion_planners::TrajOptIfoptOSQPSolverProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::motion_planners::TrajOptIfoptMoveProfile,
                                     tesseract::motion_planners::TrajOptIfoptDefaultMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::motion_planners::TrajOptIfoptCompositeProfile,
                                     tesseract::motion_planners::TrajOptIfoptDefaultCompositeProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::motion_planners::TrajOptIfoptSolverProfile,
                                     tesseract::motion_planners::TrajOptIfoptOSQPSolverProfile)

// LCOV_EXCL_START
CEREAL_REGISTER_DYNAMIC_INIT(tesseract_motion_planners_trajopt_ifopt_cereal)
// LCOV_EXCL_STOP
