#include <tesseract_time_parameterization/ruckig/cereal_serialization.h>

// These must be include before calling macro CEREAL_REGISTER_TYPE
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

CEREAL_REGISTER_TYPE(tesseract::time_parameterization::RuckigTrajectorySmoothingCompositeProfile)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile,
                                     tesseract::time_parameterization::RuckigTrajectorySmoothingCompositeProfile)

// LCOV_EXCL_START
CEREAL_REGISTER_DYNAMIC_INIT(tesseract_time_parameterization_ruckig_cereal)
// LCOV_EXCL_STOP
