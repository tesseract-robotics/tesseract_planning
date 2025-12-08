#include <tesseract_motion_planners/trajopt/cereal_serialization.h>

// These must be include before calling macro CEREAL_REGISTER_TYPE
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

CEREAL_REGISTER_TYPE(tesseract_planning::TrajOptMoveProfile)
CEREAL_REGISTER_TYPE(tesseract_planning::TrajOptCompositeProfile)
CEREAL_REGISTER_TYPE(tesseract_planning::TrajOptSolverProfile)
CEREAL_REGISTER_TYPE(tesseract_planning::TrajOptDefaultMoveProfile)
CEREAL_REGISTER_TYPE(tesseract_planning::TrajOptDefaultCompositeProfile)
CEREAL_REGISTER_TYPE(tesseract_planning::TrajOptOSQPSolverProfile)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile, tesseract_planning::TrajOptMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile, tesseract_planning::TrajOptCompositeProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile, tesseract_planning::TrajOptSolverProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile, tesseract_planning::TrajOptDefaultMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile, tesseract_planning::TrajOptDefaultCompositeProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile, tesseract_planning::TrajOptOSQPSolverProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::TrajOptMoveProfile,
                                     tesseract_planning::TrajOptDefaultMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::TrajOptCompositeProfile,
                                     tesseract_planning::TrajOptDefaultCompositeProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::TrajOptSolverProfile,
                                     tesseract_planning::TrajOptOSQPSolverProfile)

// LCOV_EXCL_START
CEREAL_REGISTER_DYNAMIC_INIT(tesseract_motion_planners_trajopt_cereal)
// LCOV_EXCL_STOP
