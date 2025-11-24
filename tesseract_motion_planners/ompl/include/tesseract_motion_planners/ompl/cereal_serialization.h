#ifndef TESSERACT_MOTION_PLANNERS_OMPL_CEREAL_SERIALIZATION_H
#define TESSERACT_MOTION_PLANNERS_OMPL_CEREAL_SERIALIZATION_H

#include <tesseract_motion_planners/ompl/ompl_solver_config.h>
#include <tesseract_motion_planners/ompl/ompl_planner_configurator.h>
#include <tesseract_motion_planners/ompl/profile/ompl_profile.h>
#include <tesseract_motion_planners/ompl/profile/ompl_real_vector_move_profile.h>

#include <tesseract_common/cereal_serialization.h>
#include <tesseract_collision/core/cereal_serialization.h>

#include <cereal/cereal.hpp>
#include <cereal/types/polymorphic.hpp>

namespace tesseract_planning
{
template <class Archive>
void serialize(Archive& /*ar*/, OMPLPlannerConfigurator& /*obj*/)
{
}

template <class Archive>
void serialize(Archive& ar, SBLConfigurator& obj)
{
  ar(cereal::base_class<OMPLPlannerConfigurator>(&obj));
  ar(cereal::make_nvp("range", obj.range));
}

template <class Archive>
void serialize(Archive& ar, ESTConfigurator& obj)
{
  ar(cereal::base_class<OMPLPlannerConfigurator>(&obj));
  ar(cereal::make_nvp("range", obj.range));
  ar(cereal::make_nvp("goal_bias", obj.goal_bias));
}

template <class Archive>
void serialize(Archive& ar, LBKPIECE1Configurator& obj)
{
  ar(cereal::base_class<OMPLPlannerConfigurator>(&obj));
  ar(cereal::make_nvp("range", obj.range));
  ar(cereal::make_nvp("border_fraction", obj.border_fraction));
  ar(cereal::make_nvp("min_valid_path_fraction", obj.min_valid_path_fraction));
}

template <class Archive>
void serialize(Archive& ar, BKPIECE1Configurator& obj)
{
  ar(cereal::base_class<OMPLPlannerConfigurator>(&obj));
  ar(cereal::make_nvp("range", obj.range));
  ar(cereal::make_nvp("border_fraction", obj.border_fraction));
  ar(cereal::make_nvp("failed_expansion_score_factor", obj.failed_expansion_score_factor));
  ar(cereal::make_nvp("min_valid_path_fraction", obj.min_valid_path_fraction));
}

template <class Archive>
void serialize(Archive& ar, KPIECE1Configurator& obj)
{
  ar(cereal::base_class<OMPLPlannerConfigurator>(&obj));
  ar(cereal::make_nvp("range", obj.range));
  ar(cereal::make_nvp("goal_bias", obj.goal_bias));
  ar(cereal::make_nvp("border_fraction", obj.border_fraction));
  ar(cereal::make_nvp("failed_expansion_score_factor", obj.failed_expansion_score_factor));
  ar(cereal::make_nvp("min_valid_path_fraction", obj.min_valid_path_fraction));
}

template <class Archive>
void serialize(Archive& ar, BiTRRTConfigurator& obj)
{
  ar(cereal::base_class<OMPLPlannerConfigurator>(&obj));
  ar(cereal::make_nvp("range", obj.range));
  ar(cereal::make_nvp("temp_change_factor", obj.temp_change_factor));
  ar(cereal::make_nvp("cost_threshold", obj.cost_threshold));
  ar(cereal::make_nvp("init_temperature", obj.init_temperature));
  ar(cereal::make_nvp("frontier_threshold", obj.frontier_threshold));
  ar(cereal::make_nvp("frontier_node_ratio", obj.frontier_node_ratio));
}

template <class Archive>
void serialize(Archive& ar, RRTConfigurator& obj)
{
  ar(cereal::base_class<OMPLPlannerConfigurator>(&obj));
  ar(cereal::make_nvp("range", obj.range));
  ar(cereal::make_nvp("goal_bias", obj.goal_bias));
}

template <class Archive>
void serialize(Archive& ar, RRTConnectConfigurator& obj)
{
  ar(cereal::base_class<OMPLPlannerConfigurator>(&obj));
  ar(cereal::make_nvp("range", obj.range));
}

template <class Archive>
void serialize(Archive& ar, RRTstarConfigurator& obj)
{
  ar(cereal::base_class<OMPLPlannerConfigurator>(&obj));
  ar(cereal::make_nvp("range", obj.range));
  ar(cereal::make_nvp("goal_bias", obj.goal_bias));
  ar(cereal::make_nvp("delay_collision_checking", obj.delay_collision_checking));
}

template <class Archive>
void serialize(Archive& ar, TRRTConfigurator& obj)
{
  ar(cereal::base_class<OMPLPlannerConfigurator>(&obj));
  ar(cereal::make_nvp("range", obj.range));
  ar(cereal::make_nvp("goal_bias", obj.goal_bias));
  ar(cereal::make_nvp("temp_change_factor", obj.temp_change_factor));
  ar(cereal::make_nvp("init_temperature", obj.init_temperature));
  ar(cereal::make_nvp("frontier_threshold", obj.frontier_threshold));
  ar(cereal::make_nvp("frontier_node_ratio", obj.frontier_node_ratio));
}

template <class Archive>
void serialize(Archive& ar, PRMConfigurator& obj)
{
  ar(cereal::base_class<OMPLPlannerConfigurator>(&obj));
  ar(cereal::make_nvp("max_nearest_neighbors", obj.max_nearest_neighbors));
}

template <class Archive>
void serialize(Archive& ar, PRMstarConfigurator& obj)
{
  ar(cereal::base_class<OMPLPlannerConfigurator>(&obj));
}

template <class Archive>
void serialize(Archive& ar, LazyPRMstarConfigurator& obj)
{
  ar(cereal::base_class<OMPLPlannerConfigurator>(&obj));
}

template <class Archive>
void serialize(Archive& ar, SPARSConfigurator& obj)
{
  ar(cereal::base_class<OMPLPlannerConfigurator>(&obj));
  ar(cereal::make_nvp("max_failures", obj.max_failures));
  ar(cereal::make_nvp("dense_delta_fraction", obj.dense_delta_fraction));
  ar(cereal::make_nvp("sparse_delta_fraction", obj.sparse_delta_fraction));
  ar(cereal::make_nvp("stretch_factor", obj.stretch_factor));
}

template <class Archive>
void serialize(Archive& ar, OMPLSolverConfig& obj)
{
  ar(cereal::make_nvp("planning_time", obj.planning_time));
  ar(cereal::make_nvp("max_solutions", obj.max_solutions));
  ar(cereal::make_nvp("simplify", obj.simplify));
  ar(cereal::make_nvp("optimize", obj.optimize));
  ar(cereal::make_nvp("planners", obj.planners));
}

template <class Archive>
void serialize(Archive& ar, OMPLMoveProfile& obj)
{
  ar(cereal::base_class<tesseract_common::Profile>(&obj));
}

template <class Archive>
void serialize(Archive& ar, OMPLRealVectorMoveProfile& obj)
{
  ar(cereal::base_class<OMPLMoveProfile>(&obj));
  ar(cereal::make_nvp("solver_config", obj.solver_config));
  ar(cereal::make_nvp("contact_manager_config", obj.contact_manager_config));
  ar(cereal::make_nvp("collision_check_config", obj.collision_check_config));
}
}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_OMPL_CEREAL_SERIALIZATION_H
