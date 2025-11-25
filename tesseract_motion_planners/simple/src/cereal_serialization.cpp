#include <tesseract_motion_planners/simple/cereal_serialization.h>

// These must be include before calling macro CEREAL_REGISTER_TYPE
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

CEREAL_REGISTER_TYPE(tesseract_planning::SimplePlannerMoveProfile)
CEREAL_REGISTER_TYPE(tesseract_planning::SimplePlannerCompositeProfile)
CEREAL_REGISTER_TYPE(tesseract_planning::SimplePlannerFixedSizeAssignMoveProfile)
CEREAL_REGISTER_TYPE(tesseract_planning::SimplePlannerFixedSizeAssignNoIKMoveProfile)
CEREAL_REGISTER_TYPE(tesseract_planning::SimplePlannerFixedSizeMoveProfile)
CEREAL_REGISTER_TYPE(tesseract_planning::SimplePlannerLVSAssignMoveProfile)
CEREAL_REGISTER_TYPE(tesseract_planning::SimplePlannerLVSAssignNoIKMoveProfile)
CEREAL_REGISTER_TYPE(tesseract_planning::SimplePlannerLVSMoveProfile)
CEREAL_REGISTER_TYPE(tesseract_planning::SimplePlannerLVSNoIKMoveProfile)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile, tesseract_planning::SimplePlannerMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile, tesseract_planning::SimplePlannerCompositeProfile)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile,
                                     tesseract_planning::SimplePlannerFixedSizeAssignMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile,
                                     tesseract_planning::SimplePlannerFixedSizeAssignNoIKMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile, tesseract_planning::SimplePlannerFixedSizeMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile, tesseract_planning::SimplePlannerLVSAssignMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile,
                                     tesseract_planning::SimplePlannerLVSAssignNoIKMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile, tesseract_planning::SimplePlannerLVSMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile, tesseract_planning::SimplePlannerLVSNoIKMoveProfile)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::SimplePlannerMoveProfile,
                                     tesseract_planning::SimplePlannerFixedSizeAssignMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::SimplePlannerMoveProfile,
                                     tesseract_planning::SimplePlannerFixedSizeAssignNoIKMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::SimplePlannerMoveProfile,
                                     tesseract_planning::SimplePlannerFixedSizeMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::SimplePlannerMoveProfile,
                                     tesseract_planning::SimplePlannerLVSAssignMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::SimplePlannerMoveProfile,
                                     tesseract_planning::SimplePlannerLVSAssignNoIKMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::SimplePlannerMoveProfile,
                                     tesseract_planning::SimplePlannerLVSMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::SimplePlannerMoveProfile,
                                     tesseract_planning::SimplePlannerLVSNoIKMoveProfile)

CEREAL_REGISTER_DYNAMIC_INIT(tesseract_motion_planners_simple_cereal)
