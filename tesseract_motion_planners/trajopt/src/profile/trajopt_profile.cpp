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
#include <typeindex>

namespace tesseract_planning
{
TrajOptMoveProfile::TrajOptMoveProfile() : Profile(TrajOptMoveProfile::getStaticKey()) {}

std::size_t TrajOptMoveProfile::getStaticKey() { return std::type_index(typeid(TrajOptMoveProfile)).hash_code(); }

TrajOptCompositeProfile::TrajOptCompositeProfile() : Profile(TrajOptCompositeProfile::getStaticKey()) {}

std::size_t TrajOptCompositeProfile::getStaticKey()
{
  return std::type_index(typeid(TrajOptCompositeProfile)).hash_code();
}

TrajOptSolverProfile::TrajOptSolverProfile() : Profile(TrajOptSolverProfile::getStaticKey()) {}

std::size_t TrajOptSolverProfile::getStaticKey() { return std::type_index(typeid(TrajOptSolverProfile)).hash_code(); }

sco::BasicTrustRegionSQPParameters TrajOptSolverProfile::createOptimizationParameters() const { return opt_params; }

std::vector<sco::Optimizer::Callback> TrajOptSolverProfile::createOptimizationCallbacks() const { return {}; }

}  // namespace tesseract_planning
