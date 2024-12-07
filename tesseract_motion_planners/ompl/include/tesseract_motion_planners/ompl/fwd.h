#ifndef TESSERACT_MOTION_PLANNERS_OMPL_FWD_H
#define TESSERACT_MOTION_PLANNERS_OMPL_FWD_H

namespace tesseract_planning
{
// compound_state_validator.h
class CompoundStateValidator;

// continuous_motion_validator.h
class ContinuousMotionValidator;

// discrete_motion_validator.h
class DiscreteMotionValidator;

// ompl_motion_planner.h
class OMPLMotionPlanner;

// ompl_planner_configurator.h
enum class OMPLPlannerType;
struct OMPLPlannerConfigurator;
struct SBLConfigurator;
struct ESTConfigurator;
struct LBKPIECE1Configurator;
struct BKPIECE1Configurator;
struct KPIECE1Configurator;
struct BiTRRTConfigurator;
struct RRTConfigurator;
struct RRTConnectConfigurator;
struct RRTstarConfigurator;
struct TRRTConfigurator;
struct PRMConfigurator;
struct PRMstarConfigurator;
struct LazyPRMstarConfigurator;
struct SPARSConfigurator;

// ompl_solver_config.h
struct OMPLSolverConfig;

// state_collision_validator.h
class StateCollisionValidator;

// weighted_real_vector_state_sampler.h
class WeightedRealVectorStateSampler;

// profile
class OMPLPlanProfile;
class OMPLRealVectorPlanProfile;
}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_OMPL_FWD_H
