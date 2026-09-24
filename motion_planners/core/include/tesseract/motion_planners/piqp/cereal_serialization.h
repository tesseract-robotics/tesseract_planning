/**
 * @file cereal_serialization.h
 * @brief Cereal serialization for PIQP settings
 *
 * @author Roelof Oomen
 * @date September 24, 2026
 *
 * @copyright Copyright (c) 2026, Roelof Oomen
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * The motion_planners component does not depend on PIQP. Include this header only from a planner component whose
 * solver library was built with PIQP; that library supplies the PIQP include path.
 */
#ifndef TESSERACT_MOTION_PLANNERS_PIQP_CEREAL_SERIALIZATION_H
#define TESSERACT_MOTION_PLANNERS_PIQP_CEREAL_SERIALIZATION_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <piqp/settings.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <cereal/cereal.hpp>
#include <cereal/types/common.hpp>

namespace piqp
{
template <class Archive, typename T>
void serialize(Archive& ar, Settings<T>& obj)
{
  ar(cereal::make_nvp("rho_init", obj.rho_init));
  ar(cereal::make_nvp("delta_init", obj.delta_init));
  ar(cereal::make_nvp("eps_abs", obj.eps_abs));
  ar(cereal::make_nvp("eps_rel", obj.eps_rel));
  ar(cereal::make_nvp("check_duality_gap", obj.check_duality_gap));
  ar(cereal::make_nvp("eps_duality_gap_abs", obj.eps_duality_gap_abs));
  ar(cereal::make_nvp("eps_duality_gap_rel", obj.eps_duality_gap_rel));
  ar(cereal::make_nvp("infeasibility_threshold", obj.infeasibility_threshold));
  ar(cereal::make_nvp("reg_lower_limit", obj.reg_lower_limit));
  ar(cereal::make_nvp("reg_finetune_lower_limit", obj.reg_finetune_lower_limit));
  ar(cereal::make_nvp("reg_finetune_primal_update_threshold", obj.reg_finetune_primal_update_threshold));
  ar(cereal::make_nvp("reg_finetune_dual_update_threshold", obj.reg_finetune_dual_update_threshold));
  ar(cereal::make_nvp("max_iter", obj.max_iter));
  ar(cereal::make_nvp("max_factor_retires", obj.max_factor_retires));
  ar(cereal::make_nvp("preconditioner_scale_cost", obj.preconditioner_scale_cost));
  ar(cereal::make_nvp("preconditioner_reuse_on_update", obj.preconditioner_reuse_on_update));
  ar(cereal::make_nvp("preconditioner_iter", obj.preconditioner_iter));
  ar(cereal::make_nvp("tau", obj.tau));
  ar(cereal::make_nvp("kkt_solver", obj.kkt_solver));
  ar(cereal::make_nvp("iterative_refinement_always_enabled", obj.iterative_refinement_always_enabled));
  ar(cereal::make_nvp("iterative_refinement_eps_abs", obj.iterative_refinement_eps_abs));
  ar(cereal::make_nvp("iterative_refinement_eps_rel", obj.iterative_refinement_eps_rel));
  ar(cereal::make_nvp("iterative_refinement_max_iter", obj.iterative_refinement_max_iter));
  ar(cereal::make_nvp("iterative_refinement_min_improvement_rate", obj.iterative_refinement_min_improvement_rate));
  ar(cereal::make_nvp("iterative_refinement_static_regularization_eps",
                      obj.iterative_refinement_static_regularization_eps));
  ar(cereal::make_nvp("iterative_refinement_static_regularization_rel",
                      obj.iterative_refinement_static_regularization_rel));
  ar(cereal::make_nvp("verbose", obj.verbose));
  ar(cereal::make_nvp("compute_timings", obj.compute_timings));
}
}  // namespace piqp

#endif  // TESSERACT_MOTION_PLANNERS_PIQP_CEREAL_SERIALIZATION_H
