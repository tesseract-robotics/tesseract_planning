#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_CEREAL_SERIALIZATION_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_CEREAL_SERIALIZATION_H

#include <trajopt_sco/optimizers.hpp>
#include <trajopt_sco/osqp_interface.hpp>
#include <trajopt_common/cereal_serialization.h>

#include <tesseract_motion_planners/trajopt/trajopt_waypoint_config.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_move_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_osqp_solver_profile.h>

#include <cereal/cereal.hpp>
#include <cereal/types/polymorphic.hpp>

namespace sco
{
template <class Archive>
void serialize(Archive& ar, BasicTrustRegionSQPParameters& obj)
{
  ar(cereal::make_nvp("improve_ratio_threshold", obj.improve_ratio_threshold));
  ar(cereal::make_nvp("min_trust_box_size", obj.min_trust_box_size));
  ar(cereal::make_nvp("min_approx_improve", obj.min_approx_improve));
  ar(cereal::make_nvp("min_approx_improve_frac", obj.min_approx_improve_frac));
  ar(cereal::make_nvp("max_iter", obj.max_iter));
  ar(cereal::make_nvp("trust_shrink_ratio", obj.trust_shrink_ratio));
  ar(cereal::make_nvp("trust_expand_ratio", obj.trust_expand_ratio));
  ar(cereal::make_nvp("cnt_tolerance", obj.cnt_tolerance));
  ar(cereal::make_nvp("max_merit_coeff_increases", obj.max_merit_coeff_increases));
  ar(cereal::make_nvp("max_qp_solver_failures", obj.max_qp_solver_failures));
  ar(cereal::make_nvp("merit_coeff_increase_ratio", obj.merit_coeff_increase_ratio));
  ar(cereal::make_nvp("max_time", obj.max_time));
  ar(cereal::make_nvp("initial_merit_error_coeff", obj.initial_merit_error_coeff));
  ar(cereal::make_nvp("inflate_constraints_individually", obj.inflate_constraints_individually));
  ar(cereal::make_nvp("trust_box_size", obj.trust_box_size));
  ar(cereal::make_nvp("log_results", obj.log_results));
  ar(cereal::make_nvp("log_dir", obj.log_dir));
  ar(cereal::make_nvp("num_threads", obj.num_threads));
}
}  // namespace sco

namespace cereal
{
template <class Archive>
void serialize(Archive& ar, OSQPSettings& obj)
{
  ar(cereal::make_nvp("rho", obj.rho));
  ar(cereal::make_nvp("sigma", obj.sigma));
  ar(cereal::make_nvp("scaling", obj.scaling));
  ar(cereal::make_nvp("adaptive_rho", obj.adaptive_rho));
  ar(cereal::make_nvp("adaptive_rho_interval", obj.adaptive_rho_interval));
  ar(cereal::make_nvp("adaptive_rho_tolerance", obj.adaptive_rho_tolerance));
  ar(cereal::make_nvp("adaptive_rho_fraction", obj.adaptive_rho_fraction));
  ar(cereal::make_nvp("max_iter", obj.max_iter));
  ar(cereal::make_nvp("eps_abs", obj.eps_abs));
  ar(cereal::make_nvp("eps_rel", obj.eps_rel));
  ar(cereal::make_nvp("eps_prim_inf", obj.eps_prim_inf));
  ar(cereal::make_nvp("eps_dual_inf", obj.eps_dual_inf));
  ar(cereal::make_nvp("alpha", obj.alpha));
  ar(cereal::make_nvp("linsys_solver", obj.linsys_solver));
  ar(cereal::make_nvp("delta", obj.delta));
  ar(cereal::make_nvp("polishing", obj.polishing));
  ar(cereal::make_nvp("polish_refine_iter", obj.polish_refine_iter));
  ar(cereal::make_nvp("verbose", obj.verbose));
  ar(cereal::make_nvp("scaled_termination", obj.scaled_termination));
  ar(cereal::make_nvp("check_termination", obj.check_termination));
  ar(cereal::make_nvp("warm_starting", obj.warm_starting));
  ar(cereal::make_nvp("time_limit", obj.time_limit));
  ar(cereal::make_nvp("allocate_solution", obj.allocate_solution));
  ar(cereal::make_nvp("cg_max_iter", obj.cg_max_iter));
  ar(cereal::make_nvp("cg_precond", obj.cg_precond));
  ar(cereal::make_nvp("cg_tol_fraction", obj.cg_tol_fraction));
  ar(cereal::make_nvp("cg_tol_reduction", obj.cg_tol_reduction));
  ar(cereal::make_nvp("check_dualgap", obj.check_dualgap));
  ar(cereal::make_nvp("device", obj.device));
  ar(cereal::make_nvp("profiler_level", obj.profiler_level));
  ar(cereal::make_nvp("rho_is_vec", obj.rho_is_vec));
}

}  // namespace cereal

namespace tesseract::motion_planners
{
template <class Archive>
void serialize(Archive& ar, TrajOptCartesianWaypointConfig& obj)
{
  ar(cereal::make_nvp("enabled", obj.enabled));
  ar(cereal::make_nvp("use_tolerance_override", obj.use_tolerance_override));
  ar(cereal::make_nvp("lower_tolerance", obj.lower_tolerance));
  ar(cereal::make_nvp("upper_tolerance", obj.upper_tolerance));
  ar(cereal::make_nvp("coeff", obj.coeff));
}

template <class Archive>
void serialize(Archive& ar, TrajOptJointWaypointConfig& obj)
{
  ar(cereal::make_nvp("enabled", obj.enabled));
  ar(cereal::make_nvp("use_tolerance_override", obj.use_tolerance_override));
  ar(cereal::make_nvp("lower_tolerance", obj.lower_tolerance));
  ar(cereal::make_nvp("upper_tolerance", obj.upper_tolerance));
  ar(cereal::make_nvp("coeff", obj.coeff));
}

template <class Archive>
void serialize(Archive& ar, TrajOptMoveProfile& obj)
{
  ar(cereal::base_class<tesseract::common::Profile>(&obj));
}

template <class Archive>
void serialize(Archive& ar, TrajOptCompositeProfile& obj)
{
  ar(cereal::base_class<tesseract::common::Profile>(&obj));
}

template <class Archive>
void serialize(Archive& ar, TrajOptSolverProfile& obj)
{
  ar(cereal::base_class<tesseract::common::Profile>(&obj));
  ar(cereal::make_nvp("opt_params", obj.opt_params));
}

template <class Archive>
void serialize(Archive& ar, TrajOptDefaultMoveProfile& obj)
{
  ar(cereal::base_class<TrajOptMoveProfile>(&obj));
  ar(cereal::make_nvp("cartesian_cost_config", obj.cartesian_cost_config));
  ar(cereal::make_nvp("cartesian_constraint_config", obj.cartesian_constraint_config));
  ar(cereal::make_nvp("joint_cost_config", obj.joint_cost_config));
  ar(cereal::make_nvp("joint_constraint_config", obj.joint_constraint_config));
}

template <class Archive>
void serialize(Archive& ar, TrajOptDefaultCompositeProfile& obj)
{
  ar(cereal::base_class<TrajOptCompositeProfile>(&obj));
  ar(cereal::make_nvp("collision_cost_config", obj.collision_cost_config));
  ar(cereal::make_nvp("collision_constraint_config", obj.collision_constraint_config));
  ar(cereal::make_nvp("smooth_velocities", obj.smooth_velocities));
  ar(cereal::make_nvp("velocity_coeff", obj.velocity_coeff));
  ar(cereal::make_nvp("smooth_accelerations", obj.smooth_accelerations));
  ar(cereal::make_nvp("acceleration_coeff", obj.acceleration_coeff));
  ar(cereal::make_nvp("smooth_jerks", obj.smooth_jerks));
  ar(cereal::make_nvp("jerk_coeff", obj.jerk_coeff));
  ar(cereal::make_nvp("avoid_singularity", obj.avoid_singularity));
  ar(cereal::make_nvp("avoid_singularity_coeff", obj.avoid_singularity_coeff));
}

template <class Archive>
void serialize(Archive& ar, TrajOptOSQPSolverProfile& obj)
{
  ar(cereal::base_class<TrajOptSolverProfile>(&obj));
  ar(cereal::make_nvp("settings", obj.settings));
  ar(cereal::make_nvp("update_workspace", obj.update_workspace));
}

}  // namespace tesseract::motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_CEREAL_SERIALIZATION_H
