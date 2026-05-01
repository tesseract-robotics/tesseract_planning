#ifndef TESSERACT_MOTION_PLANNERS_SIMPLE_CEREAL_SERIALIZATION_IMPL_HPP
#define TESSERACT_MOTION_PLANNERS_SIMPLE_CEREAL_SERIALIZATION_IMPL_HPP

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

#endif  // TESSERACT_MOTION_PLANNERS_SIMPLE_CEREAL_SERIALIZATION_IMPL_HPP
