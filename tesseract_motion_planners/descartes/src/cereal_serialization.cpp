#include <tesseract_motion_planners/descartes/cereal_serialization.h>

// These must be include before calling macro CEREAL_REGISTER_TYPE
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

CEREAL_REGISTER_TYPE(tesseract::motion_planners::DescartesSolverProfile<double>)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::DescartesSolverProfile<float>)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::DescartesMoveProfile<double>)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::DescartesMoveProfile<float>)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::DescartesLadderGraphSolverProfile<double>)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::DescartesLadderGraphSolverProfile<float>)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::DescartesDefaultMoveProfile<double>)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::DescartesDefaultMoveProfile<float>)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile,
                                     tesseract::motion_planners::DescartesSolverProfile<double>)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile,
                                     tesseract::motion_planners::DescartesSolverProfile<float>)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile,
                                     tesseract::motion_planners::DescartesMoveProfile<double>)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile,
                                     tesseract::motion_planners::DescartesMoveProfile<float>)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile,
                                     tesseract::motion_planners::DescartesLadderGraphSolverProfile<double>)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile,
                                     tesseract::motion_planners::DescartesLadderGraphSolverProfile<float>)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile,
                                     tesseract::motion_planners::DescartesDefaultMoveProfile<double>)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile,
                                     tesseract::motion_planners::DescartesDefaultMoveProfile<float>)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::motion_planners::DescartesSolverProfile<double>,
                                     tesseract::motion_planners::DescartesLadderGraphSolverProfile<double>)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::motion_planners::DescartesSolverProfile<float>,
                                     tesseract::motion_planners::DescartesLadderGraphSolverProfile<float>)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::motion_planners::DescartesMoveProfile<double>,
                                     tesseract::motion_planners::DescartesDefaultMoveProfile<double>)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::motion_planners::DescartesMoveProfile<float>,
                                     tesseract::motion_planners::DescartesDefaultMoveProfile<float>)

// LCOV_EXCL_START
CEREAL_REGISTER_DYNAMIC_INIT(tesseract_motion_planners_descartes_cereal)
// LCOV_EXCL_STOP
