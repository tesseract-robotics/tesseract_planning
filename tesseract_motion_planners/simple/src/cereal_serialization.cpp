#include <tesseract_motion_planners/simple/cereal_serialization.h>

// These must be include before calling macro CEREAL_REGISTER_TYPE
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

CEREAL_REGISTER_TYPE(tesseract::motion_planners::SimplePlannerMoveProfile)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::SimplePlannerCompositeProfile)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::SimplePlannerFixedSizeAssignMoveProfile)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::SimplePlannerFixedSizeAssignNoIKMoveProfile)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::SimplePlannerFixedSizeMoveProfile)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::SimplePlannerLVSAssignMoveProfile)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::SimplePlannerLVSAssignNoIKMoveProfile)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::SimplePlannerLVSMoveProfile)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::SimplePlannerLVSNoIKMoveProfile)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile, tesseract::motion_planners::SimplePlannerMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile,
                                     tesseract::motion_planners::SimplePlannerCompositeProfile)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile,
                                     tesseract::motion_planners::SimplePlannerFixedSizeAssignMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile,
                                     tesseract::motion_planners::SimplePlannerFixedSizeAssignNoIKMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile,
                                     tesseract::motion_planners::SimplePlannerFixedSizeMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile,
                                     tesseract::motion_planners::SimplePlannerLVSAssignMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile,
                                     tesseract::motion_planners::SimplePlannerLVSAssignNoIKMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile,
                                     tesseract::motion_planners::SimplePlannerLVSMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile,
                                     tesseract::motion_planners::SimplePlannerLVSNoIKMoveProfile)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::motion_planners::SimplePlannerMoveProfile,
                                     tesseract::motion_planners::SimplePlannerFixedSizeAssignMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::motion_planners::SimplePlannerMoveProfile,
                                     tesseract::motion_planners::SimplePlannerFixedSizeAssignNoIKMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::motion_planners::SimplePlannerMoveProfile,
                                     tesseract::motion_planners::SimplePlannerFixedSizeMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::motion_planners::SimplePlannerMoveProfile,
                                     tesseract::motion_planners::SimplePlannerLVSAssignMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::motion_planners::SimplePlannerMoveProfile,
                                     tesseract::motion_planners::SimplePlannerLVSAssignNoIKMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::motion_planners::SimplePlannerMoveProfile,
                                     tesseract::motion_planners::SimplePlannerLVSMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::motion_planners::SimplePlannerMoveProfile,
                                     tesseract::motion_planners::SimplePlannerLVSNoIKMoveProfile)

// LCOV_EXCL_START
CEREAL_REGISTER_DYNAMIC_INIT(tesseract_motion_planners_simple_cereal)
// LCOV_EXCL_STOP
