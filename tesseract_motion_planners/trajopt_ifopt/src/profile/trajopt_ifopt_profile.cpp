/**
 * @file trajopt_ifopt_profile.cpp
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
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_profile.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <typeindex>

namespace boost::serialization
{
template <class Archive>
void serialize(Archive& ar, trajopt_sqp::SQPParameters& params, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("improve_ratio_threshold", params.improve_ratio_threshold);
  ar& boost::serialization::make_nvp("min_trust_box_size", params.min_trust_box_size);
  ar& boost::serialization::make_nvp("min_approx_improve", params.min_approx_improve);
  ar& boost::serialization::make_nvp("min_approx_improve_frac", params.min_approx_improve_frac);
  ar& boost::serialization::make_nvp("max_iter", params.max_iterations);
  ar& boost::serialization::make_nvp("trust_shrink_ratio", params.trust_shrink_ratio);
  ar& boost::serialization::make_nvp("trust_expand_ratio", params.trust_expand_ratio);
  ar& boost::serialization::make_nvp("cnt_tolerance", params.cnt_tolerance);
  ar& boost::serialization::make_nvp("max_merit_coeff_increases", params.max_merit_coeff_increases);
  ar& boost::serialization::make_nvp("max_qp_solver_failures", params.max_qp_solver_failures);
  ar& boost::serialization::make_nvp("merit_coeff_increase_ratio", params.merit_coeff_increase_ratio);
  ar& boost::serialization::make_nvp("max_time", params.max_time);
  ar& boost::serialization::make_nvp("initial_merit_error_coeff", params.initial_merit_error_coeff);
  ar& boost::serialization::make_nvp("inflate_constraints_individually", params.inflate_constraints_individually);
  ar& boost::serialization::make_nvp("trust_box_size", params.initial_trust_box_size);
  ar& boost::serialization::make_nvp("log_results", params.log_results);
  ar& boost::serialization::make_nvp("log_dir", params.log_dir);
  // ar& boost::serialization::make_nvp("num_threads", params.num_threads);
}
}  // namespace boost::serialization

namespace tesseract_planning
{
TrajOptIfoptMoveProfile::TrajOptIfoptMoveProfile() : Profile(TrajOptIfoptMoveProfile::getStaticKey()) {}

std::size_t TrajOptIfoptMoveProfile::getStaticKey()
{
  return std::type_index(typeid(TrajOptIfoptMoveProfile)).hash_code();
}

template <class Archive>
void TrajOptIfoptMoveProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Profile);
}

TrajOptIfoptCompositeProfile::TrajOptIfoptCompositeProfile() : Profile(TrajOptIfoptCompositeProfile::getStaticKey()) {}

std::size_t TrajOptIfoptCompositeProfile::getStaticKey()
{
  return std::type_index(typeid(TrajOptIfoptCompositeProfile)).hash_code();
}

template <class Archive>
void TrajOptIfoptCompositeProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Profile);
}

TrajOptIfoptSolverProfile::TrajOptIfoptSolverProfile() : Profile(TrajOptIfoptSolverProfile::getStaticKey()) {}

std::size_t TrajOptIfoptSolverProfile::getStaticKey()
{
  return std::type_index(typeid(TrajOptIfoptSolverProfile)).hash_code();
}

template <class Archive>
void TrajOptIfoptSolverProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Profile);
  ar& BOOST_SERIALIZATION_NVP(opt_params);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TrajOptIfoptMoveProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TrajOptIfoptMoveProfile)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TrajOptIfoptCompositeProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TrajOptIfoptCompositeProfile)
TESSERACT_SERIALIZE_FREE_ARCHIVES_INSTANTIATE(trajopt_sqp::SQPParameters)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TrajOptIfoptSolverProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TrajOptIfoptSolverProfile)
