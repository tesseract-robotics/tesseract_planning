#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_CEREAL_SERIALIZATION_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_CEREAL_SERIALIZATION_H

#include <OsqpEigen/Settings.hpp>
#include <trajopt_sqp/types.h>
#include <trajopt_common/cereal_serialization.h>
#include <tesseract_common/cereal_serialization.h>
#include <tesseract_collision/core/cereal_serialization.h>

#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_waypoint_config.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_move_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_osqp_solver_profile.h>

#include <cereal/cereal.hpp>
#include <cereal/types/polymorphic.hpp>

namespace OsqpEigen
{
template <class Archive>
void serialize(Archive& ar, Settings& obj)
{
  OSQPSettings& settings = *obj.getSettings();
  ar(cereal::make_nvp("rho", settings.rho));
  ar(cereal::make_nvp("sigma", settings.sigma));
  ar(cereal::make_nvp("scaling", settings.scaling));
  ar(cereal::make_nvp("adaptive_rho", settings.adaptive_rho));
  ar(cereal::make_nvp("adaptive_rho_interval", settings.adaptive_rho_interval));
  ar(cereal::make_nvp("adaptive_rho_tolerance", settings.adaptive_rho_tolerance));
  ar(cereal::make_nvp("adaptive_rho_fraction", settings.adaptive_rho_fraction));
  ar(cereal::make_nvp("max_iter", settings.max_iter));
  ar(cereal::make_nvp("eps_abs", settings.eps_abs));
  ar(cereal::make_nvp("eps_rel", settings.eps_rel));
  ar(cereal::make_nvp("eps_prim_inf", settings.eps_prim_inf));
  ar(cereal::make_nvp("eps_dual_inf", settings.eps_dual_inf));
  ar(cereal::make_nvp("alpha", settings.alpha));
  ar(cereal::make_nvp("linsys_solver", settings.linsys_solver));
  ar(cereal::make_nvp("delta", settings.delta));
  ar(cereal::make_nvp("polishing", settings.polishing));
  ar(cereal::make_nvp("polish_refine_iter", settings.polish_refine_iter));
  ar(cereal::make_nvp("verbose", settings.verbose));
  ar(cereal::make_nvp("scaled_termination", settings.scaled_termination));
  ar(cereal::make_nvp("check_termination", settings.check_termination));
  ar(cereal::make_nvp("warm_starting", settings.warm_starting));
  ar(cereal::make_nvp("time_limit", settings.time_limit));
  ar(cereal::make_nvp("allocate_solution", settings.allocate_solution));
  ar(cereal::make_nvp("cg_max_iter", settings.cg_max_iter));
  ar(cereal::make_nvp("cg_precond", settings.cg_precond));
  ar(cereal::make_nvp("cg_tol_fraction", settings.cg_tol_fraction));
  ar(cereal::make_nvp("cg_tol_reduction", settings.cg_tol_reduction));
  ar(cereal::make_nvp("check_dualgap", settings.check_dualgap));
  ar(cereal::make_nvp("device", settings.device));
  ar(cereal::make_nvp("profiler_level", settings.profiler_level));
  ar(cereal::make_nvp("rho_is_vec", settings.rho_is_vec));
}
}  // namespace OsqpEigen

namespace trajopt_sqp
{
template <class Archive>
void serialize(Archive& ar, SQPParameters& obj)
{
  ar(cereal::make_nvp("improve_ratio_threshold", obj.improve_ratio_threshold));
  ar(cereal::make_nvp("min_trust_box_size", obj.min_trust_box_size));
  ar(cereal::make_nvp("min_approx_improve", obj.min_approx_improve));
  ar(cereal::make_nvp("min_approx_improve_frac", obj.min_approx_improve_frac));
  ar(cereal::make_nvp("max_iter", obj.max_iterations));
  ar(cereal::make_nvp("trust_shrink_ratio", obj.trust_shrink_ratio));
  ar(cereal::make_nvp("trust_expand_ratio", obj.trust_expand_ratio));
  ar(cereal::make_nvp("cnt_tolerance", obj.cnt_tolerance));
  ar(cereal::make_nvp("max_merit_coeff_increases", obj.max_merit_coeff_increases));
  ar(cereal::make_nvp("max_qp_solver_failures", obj.max_qp_solver_failures));
  ar(cereal::make_nvp("merit_coeff_increase_ratio", obj.merit_coeff_increase_ratio));
  ar(cereal::make_nvp("max_time", obj.max_time));
  ar(cereal::make_nvp("initial_merit_error_coeff", obj.initial_merit_error_coeff));
  ar(cereal::make_nvp("inflate_constraints_individually", obj.inflate_constraints_individually));
  ar(cereal::make_nvp("trust_box_size", obj.initial_trust_box_size));
  ar(cereal::make_nvp("log_results", obj.log_results));
  ar(cereal::make_nvp("log_dir", obj.log_dir));
  // ar(cereal::make_nvp("num_threads", obj.num_threads));
}
}  // namespace trajopt_sqp

namespace tesseract::motion_planners
{
template <class Archive>
void serialize(Archive& ar, TrajOptIfoptCartesianWaypointConfig& obj)
{
  ar(cereal::make_nvp("enabled", obj.enabled));
  ar(cereal::make_nvp("use_tolerance_override", obj.use_tolerance_override));
  ar(cereal::make_nvp("lower_tolerance", obj.lower_tolerance));
  ar(cereal::make_nvp("upper_tolerance", obj.upper_tolerance));
  ar(cereal::make_nvp("coeff", obj.coeff));
}

template <class Archive>
void serialize(Archive& ar, TrajOptIfoptJointWaypointConfig& obj)
{
  ar(cereal::make_nvp("enabled", obj.enabled));
  ar(cereal::make_nvp("use_tolerance_override", obj.use_tolerance_override));
  ar(cereal::make_nvp("lower_tolerance", obj.lower_tolerance));
  ar(cereal::make_nvp("upper_tolerance", obj.upper_tolerance));
  ar(cereal::make_nvp("coeff", obj.coeff));
}

template <class Archive>
void serialize(Archive& ar, TrajOptIfoptMoveProfile& obj)
{
  ar(cereal::base_class<tesseract::common::Profile>(&obj));
}

template <class Archive>
void serialize(Archive& ar, TrajOptIfoptCompositeProfile& obj)
{
  ar(cereal::base_class<tesseract::common::Profile>(&obj));
}

template <class Archive>
void serialize(Archive& ar, TrajOptIfoptSolverProfile& obj)
{
  ar(cereal::base_class<tesseract::common::Profile>(&obj));
  ar(cereal::make_nvp("opt_params", obj.opt_params));
}

template <class Archive>
void serialize(Archive& ar, TrajOptIfoptDefaultMoveProfile& obj)
{
  ar(cereal::base_class<TrajOptIfoptMoveProfile>(&obj));
  ar(cereal::make_nvp("cartesian_cost_config", obj.cartesian_cost_config));
  ar(cereal::make_nvp("cartesian_constraint_config", obj.cartesian_constraint_config));
  ar(cereal::make_nvp("joint_cost_config", obj.joint_cost_config));
  ar(cereal::make_nvp("joint_constraint_config", obj.joint_constraint_config));
}

template <class Archive>
void serialize(Archive& ar, TrajOptIfoptDefaultCompositeProfile& obj)
{
  ar(cereal::base_class<TrajOptIfoptCompositeProfile>(&obj));
  ar(cereal::make_nvp("collision_cost_config", obj.collision_cost_config));
  ar(cereal::make_nvp("collision_constraint_config", obj.collision_constraint_config));
  ar(cereal::make_nvp("smooth_velocities", obj.smooth_velocities));
  ar(cereal::make_nvp("velocity_coeff", obj.velocity_coeff));
  ar(cereal::make_nvp("smooth_accelerations", obj.smooth_accelerations));
  ar(cereal::make_nvp("acceleration_coeff", obj.acceleration_coeff));
  ar(cereal::make_nvp("smooth_jerks", obj.smooth_jerks));
  ar(cereal::make_nvp("jerk_coeff", obj.jerk_coeff));
}

template <class Archive>
void serialize(Archive& ar, TrajOptIfoptOSQPSolverProfile& obj)
{
  ar(cereal::base_class<TrajOptIfoptSolverProfile>(&obj));
  ar(cereal::make_nvp("qp_settings", obj.qp_settings));
}

}  // namespace tesseract::motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_CEREAL_SERIALIZATION_H
