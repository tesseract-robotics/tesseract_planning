#include <tesseract_time_parameterization/totg/cereal_serialization.h>

// These must be include before calling macro CEREAL_REGISTER_TYPE
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

CEREAL_REGISTER_TYPE(tesseract_planning::TimeOptimalTrajectoryGenerationCompositeProfile)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile,
                                     tesseract_planning::TimeOptimalTrajectoryGenerationCompositeProfile)

CEREAL_REGISTER_DYNAMIC_INIT(tesseract_time_parameterization_totg_cereal)
