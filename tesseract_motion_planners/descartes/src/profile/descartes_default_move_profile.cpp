/**
 * @file descartes_default_move_profile.cpp
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
#include <tesseract_motion_planners/descartes/impl/profile/descartes_default_move_profile.hpp>
#include <tesseract_motion_planners/descartes/descartes_vertex_evaluator.h>
#include <tesseract_collision/core/serialization.h>
#include <tesseract_common/eigen_serialization.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/shared_ptr.hpp>

namespace tesseract_planning
{
// Explicit template instantiation
template class DescartesDefaultMoveProfile<float>;
template class DescartesDefaultMoveProfile<double>;

template <typename FloatType>
template <class Archive>
void DescartesDefaultMoveProfile<FloatType>::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(DescartesMoveProfile<FloatType>);
  ar& BOOST_SERIALIZATION_NVP(target_pose_fixed);
  ar& BOOST_SERIALIZATION_NVP(target_pose_sample_axis);
  ar& BOOST_SERIALIZATION_NVP(target_pose_sample_resolution);
  ar& BOOST_SERIALIZATION_NVP(target_pose_sample_min);
  ar& BOOST_SERIALIZATION_NVP(target_pose_sample_max);
  ar& BOOST_SERIALIZATION_NVP(manipulator_ik_solver);
  ar& BOOST_SERIALIZATION_NVP(allow_collision);
  ar& BOOST_SERIALIZATION_NVP(enable_collision);
  ar& BOOST_SERIALIZATION_NVP(vertex_contact_manager_config);
  ar& BOOST_SERIALIZATION_NVP(vertex_collision_check_config);
  ar& BOOST_SERIALIZATION_NVP(enable_edge_collision);
  ar& BOOST_SERIALIZATION_NVP(edge_contact_manager_config);
  ar& BOOST_SERIALIZATION_NVP(edge_collision_check_config);
  ar& BOOST_SERIALIZATION_NVP(use_redundant_joint_solutions);
  ar& BOOST_SERIALIZATION_NVP(debug);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::DescartesDefaultMoveProfile<float>)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::DescartesDefaultMoveProfile<float>)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::DescartesDefaultMoveProfile<double>)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::DescartesDefaultMoveProfile<double>)
