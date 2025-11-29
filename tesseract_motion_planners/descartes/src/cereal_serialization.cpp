#include <tesseract_motion_planners/descartes/cereal_serialization.h>

// These must be include before calling macro CEREAL_REGISTER_TYPE
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

CEREAL_REGISTER_TYPE(tesseract_planning::DescartesSolverProfile<double>)
CEREAL_REGISTER_TYPE(tesseract_planning::DescartesSolverProfile<float>)
CEREAL_REGISTER_TYPE(tesseract_planning::DescartesMoveProfile<double>)
CEREAL_REGISTER_TYPE(tesseract_planning::DescartesMoveProfile<float>)
CEREAL_REGISTER_TYPE(tesseract_planning::DescartesLadderGraphSolverProfile<double>)
CEREAL_REGISTER_TYPE(tesseract_planning::DescartesLadderGraphSolverProfile<float>)
CEREAL_REGISTER_TYPE(tesseract_planning::DescartesDefaultMoveProfile<double>)
CEREAL_REGISTER_TYPE(tesseract_planning::DescartesDefaultMoveProfile<float>)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile, tesseract_planning::DescartesSolverProfile<double>)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile, tesseract_planning::DescartesSolverProfile<float>)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile, tesseract_planning::DescartesMoveProfile<double>)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile, tesseract_planning::DescartesMoveProfile<float>)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile,
                                     tesseract_planning::DescartesLadderGraphSolverProfile<double>)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile,
                                     tesseract_planning::DescartesLadderGraphSolverProfile<float>)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile, tesseract_planning::DescartesDefaultMoveProfile<double>)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile, tesseract_planning::DescartesDefaultMoveProfile<float>)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::DescartesSolverProfile<double>,
                                     tesseract_planning::DescartesLadderGraphSolverProfile<double>)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::DescartesSolverProfile<float>,
                                     tesseract_planning::DescartesLadderGraphSolverProfile<float>)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::DescartesMoveProfile<double>,
                                     tesseract_planning::DescartesDefaultMoveProfile<double>)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::DescartesMoveProfile<float>,
                                     tesseract_planning::DescartesDefaultMoveProfile<float>)

// LCOV_EXCL_START
CEREAL_REGISTER_DYNAMIC_INIT(tesseract_motion_planners_descartes_cereal)
// LCOV_EXCL_STOP
