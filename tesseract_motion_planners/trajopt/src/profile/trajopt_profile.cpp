/**
 * @file trajopt_profile.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date June 18, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
 */
#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <typeindex>

namespace boost::serialization
{
template <class Archive>
void serialize(Archive& ar, sco::BasicTrustRegionSQPParameters& params, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("improve_ratio_threshold", params.improve_ratio_threshold);
  ar& boost::serialization::make_nvp("min_trust_box_size", params.min_trust_box_size);
  ar& boost::serialization::make_nvp("min_approx_improve", params.min_approx_improve);
  ar& boost::serialization::make_nvp("min_approx_improve_frac", params.min_approx_improve_frac);
  ar& boost::serialization::make_nvp("max_iter", params.max_iter);
  ar& boost::serialization::make_nvp("trust_shrink_ratio", params.trust_shrink_ratio);
  ar& boost::serialization::make_nvp("trust_expand_ratio", params.trust_expand_ratio);
  ar& boost::serialization::make_nvp("cnt_tolerance", params.cnt_tolerance);
  ar& boost::serialization::make_nvp("max_merit_coeff_increases", params.max_merit_coeff_increases);
  ar& boost::serialization::make_nvp("max_qp_solver_failures", params.max_qp_solver_failures);
  ar& boost::serialization::make_nvp("merit_coeff_increase_ratio", params.merit_coeff_increase_ratio);
  ar& boost::serialization::make_nvp("max_time", params.max_time);
  ar& boost::serialization::make_nvp("initial_merit_error_coeff", params.initial_merit_error_coeff);
  ar& boost::serialization::make_nvp("inflate_constraints_individually", params.inflate_constraints_individually);
  ar& boost::serialization::make_nvp("trust_box_size", params.trust_box_size);
  ar& boost::serialization::make_nvp("log_results", params.log_results);
  ar& boost::serialization::make_nvp("log_dir", params.log_dir);
  ar& boost::serialization::make_nvp("num_threads", params.num_threads);
}
}  // namespace boost::serialization

namespace tesseract_planning
{
TrajOptPlanProfile::TrajOptPlanProfile() : Profile(TrajOptPlanProfile::getStaticKey()) {}

std::size_t TrajOptPlanProfile::getStaticKey() { return std::type_index(typeid(TrajOptPlanProfile)).hash_code(); }

template <class Archive>
void TrajOptPlanProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Profile);
}

TrajOptCompositeProfile::TrajOptCompositeProfile() : Profile(TrajOptCompositeProfile::getStaticKey()) {}

std::size_t TrajOptCompositeProfile::getStaticKey()
{
  return std::type_index(typeid(TrajOptCompositeProfile)).hash_code();
}

template <class Archive>
void TrajOptCompositeProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Profile);
}

TrajOptSolverProfile::TrajOptSolverProfile() : Profile(TrajOptSolverProfile::getStaticKey()) {}

std::size_t TrajOptSolverProfile::getStaticKey() { return std::type_index(typeid(TrajOptSolverProfile)).hash_code(); }

sco::BasicTrustRegionSQPParameters TrajOptSolverProfile::createOptimizationParameters() const { return opt_params; }

std::vector<sco::Optimizer::Callback> TrajOptSolverProfile::createOptimizationCallbacks() const { return {}; }

template <class Archive>
void TrajOptSolverProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Profile);
  ar& BOOST_SERIALIZATION_NVP(opt_params);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_FREE_ARCHIVES_INSTANTIATE(sco::BasicTrustRegionSQPParameters)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TrajOptPlanProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TrajOptPlanProfile)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TrajOptCompositeProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TrajOptCompositeProfile)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TrajOptSolverProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TrajOptSolverProfile)
