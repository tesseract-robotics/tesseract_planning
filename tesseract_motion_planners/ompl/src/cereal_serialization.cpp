#include <tesseract_motion_planners/ompl/cereal_serialization.h>

// These must be include before calling macro CEREAL_REGISTER_TYPE
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

CEREAL_REGISTER_TYPE(tesseract_planning::SBLConfigurator)
CEREAL_REGISTER_TYPE(tesseract_planning::ESTConfigurator)
CEREAL_REGISTER_TYPE(tesseract_planning::LBKPIECE1Configurator)
CEREAL_REGISTER_TYPE(tesseract_planning::BKPIECE1Configurator)
CEREAL_REGISTER_TYPE(tesseract_planning::KPIECE1Configurator)
CEREAL_REGISTER_TYPE(tesseract_planning::BiTRRTConfigurator)
CEREAL_REGISTER_TYPE(tesseract_planning::RRTConfigurator)
CEREAL_REGISTER_TYPE(tesseract_planning::RRTConnectConfigurator)
CEREAL_REGISTER_TYPE(tesseract_planning::RRTstarConfigurator)
CEREAL_REGISTER_TYPE(tesseract_planning::TRRTConfigurator)
CEREAL_REGISTER_TYPE(tesseract_planning::PRMConfigurator)
CEREAL_REGISTER_TYPE(tesseract_planning::PRMstarConfigurator)
CEREAL_REGISTER_TYPE(tesseract_planning::LazyPRMstarConfigurator)
CEREAL_REGISTER_TYPE(tesseract_planning::SPARSConfigurator)
CEREAL_REGISTER_TYPE(tesseract_planning::OMPLMoveProfile)
CEREAL_REGISTER_TYPE(tesseract_planning::OMPLRealVectorMoveProfile)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::OMPLPlannerConfigurator, tesseract_planning::SBLConfigurator)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::OMPLPlannerConfigurator, tesseract_planning::ESTConfigurator)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::OMPLPlannerConfigurator,
                                     tesseract_planning::LBKPIECE1Configurator)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::OMPLPlannerConfigurator,
                                     tesseract_planning::BKPIECE1Configurator)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::OMPLPlannerConfigurator,
                                     tesseract_planning::KPIECE1Configurator)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::OMPLPlannerConfigurator,
                                     tesseract_planning::BiTRRTConfigurator)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::OMPLPlannerConfigurator, tesseract_planning::RRTConfigurator)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::OMPLPlannerConfigurator,
                                     tesseract_planning::RRTConnectConfigurator)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::OMPLPlannerConfigurator,
                                     tesseract_planning::RRTstarConfigurator)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::OMPLPlannerConfigurator, tesseract_planning::TRRTConfigurator)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::OMPLPlannerConfigurator, tesseract_planning::PRMConfigurator)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::OMPLPlannerConfigurator,
                                     tesseract_planning::PRMstarConfigurator)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::OMPLPlannerConfigurator,
                                     tesseract_planning::LazyPRMstarConfigurator)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::OMPLPlannerConfigurator, tesseract_planning::SPARSConfigurator)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile, tesseract_planning::OMPLMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile, tesseract_planning::OMPLRealVectorMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::OMPLMoveProfile, tesseract_planning::OMPLRealVectorMoveProfile)

CEREAL_REGISTER_DYNAMIC_INIT(tesseract_motion_planners_ompl_cereal)
