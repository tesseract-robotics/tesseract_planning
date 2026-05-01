#ifndef TESSERACT_MOTION_PLANNERS_OMPL_CEREAL_SERIALIZATION_IMPL_HPP
#define TESSERACT_MOTION_PLANNERS_OMPL_CEREAL_SERIALIZATION_IMPL_HPP

// These must be include before calling macro CEREAL_REGISTER_TYPE
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

CEREAL_REGISTER_TYPE(tesseract::motion_planners::SBLConfigurator)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::ESTConfigurator)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::LBKPIECE1Configurator)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::BKPIECE1Configurator)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::KPIECE1Configurator)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::BiTRRTConfigurator)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::RRTConfigurator)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::RRTConnectConfigurator)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::RRTstarConfigurator)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::TRRTConfigurator)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::PRMConfigurator)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::PRMstarConfigurator)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::LazyPRMstarConfigurator)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::SPARSConfigurator)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::OMPLMoveProfile)
CEREAL_REGISTER_TYPE(tesseract::motion_planners::OMPLRealVectorMoveProfile)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::motion_planners::OMPLPlannerConfigurator,
                                     tesseract::motion_planners::SBLConfigurator)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::motion_planners::OMPLPlannerConfigurator,
                                     tesseract::motion_planners::ESTConfigurator)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::motion_planners::OMPLPlannerConfigurator,
                                     tesseract::motion_planners::LBKPIECE1Configurator)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::motion_planners::OMPLPlannerConfigurator,
                                     tesseract::motion_planners::BKPIECE1Configurator)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::motion_planners::OMPLPlannerConfigurator,
                                     tesseract::motion_planners::KPIECE1Configurator)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::motion_planners::OMPLPlannerConfigurator,
                                     tesseract::motion_planners::BiTRRTConfigurator)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::motion_planners::OMPLPlannerConfigurator,
                                     tesseract::motion_planners::RRTConfigurator)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::motion_planners::OMPLPlannerConfigurator,
                                     tesseract::motion_planners::RRTConnectConfigurator)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::motion_planners::OMPLPlannerConfigurator,
                                     tesseract::motion_planners::RRTstarConfigurator)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::motion_planners::OMPLPlannerConfigurator,
                                     tesseract::motion_planners::TRRTConfigurator)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::motion_planners::OMPLPlannerConfigurator,
                                     tesseract::motion_planners::PRMConfigurator)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::motion_planners::OMPLPlannerConfigurator,
                                     tesseract::motion_planners::PRMstarConfigurator)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::motion_planners::OMPLPlannerConfigurator,
                                     tesseract::motion_planners::LazyPRMstarConfigurator)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::motion_planners::OMPLPlannerConfigurator,
                                     tesseract::motion_planners::SPARSConfigurator)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile, tesseract::motion_planners::OMPLMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile, tesseract::motion_planners::OMPLRealVectorMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::motion_planners::OMPLMoveProfile,
                                     tesseract::motion_planners::OMPLRealVectorMoveProfile)

#endif  // TESSERACT_MOTION_PLANNERS_OMPL_CEREAL_SERIALIZATION_IMPL_HPP
