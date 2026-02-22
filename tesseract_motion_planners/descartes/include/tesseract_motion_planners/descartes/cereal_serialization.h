#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_CEREAL_SERIALIZATION_H
#define TESSERACT_MOTION_PLANNERS_DESCARTES_CEREAL_SERIALIZATION_H

#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_move_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_ladder_graph_solver_profile.h>

#include <tesseract_common/cereal_serialization.h>
#include <tesseract_collision/core/cereal_serialization.h>

#include <cereal/cereal.hpp>
#include <cereal/types/polymorphic.hpp>

namespace tesseract::motion_planners
{
template <class Archive, typename FloatType>
void serialize(Archive& ar, DescartesSolverProfile<FloatType>& obj)
{
  ar(cereal::base_class<tesseract::common::Profile>(&obj));
}

template <class Archive, typename FloatType>
void serialize(Archive& ar, DescartesMoveProfile<FloatType>& obj)
{
  ar(cereal::base_class<tesseract::common::Profile>(&obj));
}

template <class Archive, typename FloatType>
void serialize(Archive& ar, DescartesLadderGraphSolverProfile<FloatType>& obj)
{
  ar(cereal::base_class<DescartesSolverProfile<FloatType>>(&obj));
  ar(cereal::make_nvp("num_threads", obj.num_threads));
}

template <class Archive, typename FloatType>
void serialize(Archive& ar, DescartesDefaultMoveProfile<FloatType>& obj)
{
  ar(cereal::base_class<DescartesMoveProfile<FloatType>>(&obj));
  ar(cereal::make_nvp("target_pose_fixed", obj.target_pose_fixed));
  ar(cereal::make_nvp("target_pose_sample_axis", obj.target_pose_sample_axis));
  ar(cereal::make_nvp("target_pose_sample_resolution", obj.target_pose_sample_resolution));
  ar(cereal::make_nvp("target_pose_sample_min", obj.target_pose_sample_min));
  ar(cereal::make_nvp("target_pose_sample_max", obj.target_pose_sample_max));
  ar(cereal::make_nvp("manipulator_ik_solver", obj.manipulator_ik_solver));
  ar(cereal::make_nvp("allow_collision", obj.allow_collision));
  ar(cereal::make_nvp("enable_collision", obj.enable_collision));
  ar(cereal::make_nvp("vertex_contact_manager_config", obj.vertex_contact_manager_config));
  ar(cereal::make_nvp("vertex_collision_check_config", obj.vertex_collision_check_config));
  ar(cereal::make_nvp("enable_edge_collision", obj.enable_edge_collision));
  ar(cereal::make_nvp("edge_contact_manager_config", obj.edge_contact_manager_config));
  ar(cereal::make_nvp("edge_collision_check_config", obj.edge_collision_check_config));
  ar(cereal::make_nvp("use_redundant_joint_solutions", obj.use_redundant_joint_solutions));
  ar(cereal::make_nvp("debug", obj.debug));
}

}  // namespace tesseract::motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_CEREAL_SERIALIZATION_H
