#ifndef TESSERACT_TIME_PARAMETERIZATION_ISP_CEREAL_SERIALIZATION_IMPL_HPP
#define TESSERACT_TIME_PARAMETERIZATION_ISP_CEREAL_SERIALIZATION_IMPL_HPP

// These must be include before calling macro CEREAL_REGISTER_TYPE
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

CEREAL_REGISTER_TYPE(tesseract::time_parameterization::IterativeSplineParameterizationCompositeProfile)
CEREAL_REGISTER_TYPE(tesseract::time_parameterization::IterativeSplineParameterizationMoveProfile)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile,
                                     tesseract::time_parameterization::IterativeSplineParameterizationCompositeProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile,
                                     tesseract::time_parameterization::IterativeSplineParameterizationMoveProfile)

#endif  // TESSERACT_TIME_PARAMETERIZATION_ISP_CEREAL_SERIALIZATION_IMPL_HPP
