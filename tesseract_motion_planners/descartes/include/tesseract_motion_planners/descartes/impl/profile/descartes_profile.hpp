/**
 * @file descartes_profile.h
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
#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_IMPL_DESCARTES_PROFILE_HPP
#define TESSERACT_MOTION_PLANNERS_DESCARTES_IMPL_DESCARTES_PROFILE_HPP
#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_environment/environment.h>
#include <tesseract_kinematics/core/kinematic_group.h>

namespace tesseract_planning
{
template <typename FloatType>
std::shared_ptr<const tesseract_kinematics::KinematicGroup>
DescartesPlanProfile<FloatType>::createKinematicGroup(const tesseract_common::ManipulatorInfo& manip_info,
                                                      const tesseract_environment::Environment& env) const
{
  // Get Manipulator Information
  try
  {
    if (manip_info.manipulator_ik_solver.empty())
      return env.getKinematicGroup(manip_info.manipulator);

    return env.getKinematicGroup(manip_info.manipulator, manip_info.manipulator_ik_solver);
  }
  catch (...)
  {
    throw std::runtime_error("Descartes problem generator failed to create kinematic group!");
  }
}

}  // namespace tesseract_planning
#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_IMPL_DESCARTES_PROFILE_HPP
