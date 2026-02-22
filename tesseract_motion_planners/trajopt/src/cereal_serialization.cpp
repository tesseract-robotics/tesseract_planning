#include <tesseract_motion_planners/trajopt/cereal_serialization.h>

// These must be include before calling macro CEREAL_REGISTER_TYPE
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

CEREAL_REGISTER_TYPE(tesseract::motion_planners::TrajOptMoveProfile)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::TrajOptCompositeProfile)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::TrajOptSolverProfile)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::TrajOptDefaultMoveProfile)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::TrajOptDefaultCompositeProfile)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::TrajOptOSQPSolverProfile)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile, tesseract::motion_planners::TrajOptMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile, tesseract::motion_planners::TrajOptCompositeProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile, tesseract::motion_planners::TrajOptSolverProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile, tesseract::motion_planners::TrajOptDefaultMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile,
                                     tesseract::motion_planners::TrajOptDefaultCompositeProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile, tesseract::motion_planners::TrajOptOSQPSolverProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::motion_planners::TrajOptMoveProfile,
                                     tesseract::motion_planners::TrajOptDefaultMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::motion_planners::TrajOptCompositeProfile,
                                     tesseract::motion_planners::TrajOptDefaultCompositeProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::motion_planners::TrajOptSolverProfile,
                                     tesseract::motion_planners::TrajOptOSQPSolverProfile)

// LCOV_EXCL_START
CEREAL_REGISTER_DYNAMIC_INIT(tesseract_motion_planners_trajopt_cereal)
// LCOV_EXCL_STOP
