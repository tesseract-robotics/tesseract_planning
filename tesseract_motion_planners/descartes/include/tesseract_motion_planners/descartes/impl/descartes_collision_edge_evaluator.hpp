/**
 * @file descartes_collision_edge_evaluator.hpp
 * @brief Tesseract Descartes Collision Edge Evaluator Implementation
 *
 * @author Levi Armstrong
 * @date December 18, 2019
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_COLLISION_EDGE_EVALUATOR_HPP
#define TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_COLLISION_EDGE_EVALUATOR_HPP

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <thread>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/descartes/descartes_collision_edge_evaluator.h>

#include <tesseract/kinematics/joint_group.h>
#include <tesseract/collision/discrete_contact_manager.h>
#include <tesseract/collision/continuous_contact_manager.h>
#include <tesseract/environment/environment.h>
#include <tesseract/environment/utils.h>

namespace tesseract::motion_planners
{
template <typename FloatType>
DescartesCollisionEdgeEvaluator<FloatType>::DescartesCollisionEdgeEvaluator(
    const tesseract::environment::Environment& collision_env,
    std::shared_ptr<const tesseract::kinematics::JointGroup> manip,
    const tesseract::collision::ContactManagerConfig& contact_manager_config,
    tesseract::collision::CollisionCheckConfig collision_check_config,
    bool allow_collision,
    bool debug)
  : manip_(std::move(manip))
  , active_link_names_(manip_->getActiveLinkNames())
  , discrete_contact_manager_(collision_env.getDiscreteContactManager())
  , continuous_contact_manager_(collision_env.getContinuousContactManager())
  , collision_check_config_(std::move(collision_check_config))
  , allow_collision_(allow_collision)
  , debug_(debug)
{
  collision_check_config_.contact_request.type = (allow_collision_) ? tesseract::collision::ContactTestType::CLOSEST :
                                                                      tesseract::collision::ContactTestType::FIRST;

  if (discrete_contact_manager_ != nullptr)
  {
    discrete_contact_manager_->setActiveCollisionObjects(active_link_names_);
    discrete_contact_manager_->applyContactManagerConfig(contact_manager_config);
    contact_margin_data_ = discrete_contact_manager_->getCollisionMarginData();
  }
  else if (collision_check_config_.type == tesseract::collision::CollisionEvaluatorType::DISCRETE ||
           collision_check_config_.type == tesseract::collision::CollisionEvaluatorType::LVS_DISCRETE)
  {
    throw std::runtime_error("Evaluator type is DISCRETE or LVS_DISCRETE, but discrete contact manager is not "
                             "available");
  }

  if (continuous_contact_manager_ != nullptr)
  {
    continuous_contact_manager_->setActiveCollisionObjects(active_link_names_);
    continuous_contact_manager_->applyContactManagerConfig(contact_manager_config);
    contact_margin_data_ = discrete_contact_manager_->getCollisionMarginData();
  }
  else if (collision_check_config_.type == tesseract::collision::CollisionEvaluatorType::CONTINUOUS ||
           collision_check_config_.type == tesseract::collision::CollisionEvaluatorType::LVS_CONTINUOUS)
  {
    throw std::runtime_error("Evaluator type is CONTINUOUS or LVS_CONTINUOUS, but continuous contact manager is not "
                             "available");
  }
}

template <typename FloatType>
std::pair<bool, FloatType>
DescartesCollisionEdgeEvaluator<FloatType>::evaluate(const descartes_light::State<FloatType>& start,
                                                     const descartes_light::State<FloatType>& end) const
{
  assert(start.values.rows() == end.values.rows());

  // Happens in two phases:
  // 1. Compute the transform of all objects
  tesseract::common::TrajArray segment(2, start.values.rows());
  for (Eigen::Index i = 0; i < start.values.rows(); ++i)
  {
    segment(0, i) = start[i];
    segment(1, i) = end[i];
  }

  std::vector<tesseract::collision::ContactResultMap> contact_results;
  bool in_contact{ true };
  if (collision_check_config_.type == tesseract::collision::CollisionEvaluatorType::CONTINUOUS ||
      collision_check_config_.type == tesseract::collision::CollisionEvaluatorType::LVS_CONTINUOUS)
  {
    in_contact = continuousCollisionCheck(contact_results, segment);
  }
  else
  {
    in_contact = discreteCollisionCheck(contact_results, segment);
  }

  if (!in_contact)
    return std::make_pair(true, 0);

  if (!allow_collision_)
    return std::make_pair(false, 0);

  // Compute worst cost
  double cost{ std::numeric_limits<double>::lowest() };
  for (const auto& contact_result_map : contact_results)
  {
    for (const auto& pair : contact_result_map)
    {
      const double margin = contact_margin_data_.getCollisionMargin(pair.first.first, pair.first.second);
      for (const auto& contact_result : pair.second)
        cost = std::max(cost, margin - contact_result.distance);
    }
  }

  return std::make_pair(true, cost);
}

template <typename FloatType>
bool DescartesCollisionEdgeEvaluator<FloatType>::continuousCollisionCheck(
    std::vector<tesseract::collision::ContactResultMap>& results,
    const tesseract::common::TrajArray& segment) const
{
  // It was time using chronos time elapsed and it was faster to cache the contact manager
  unsigned long int hash = std::hash<std::thread::id>{}(std::this_thread::get_id());
  tesseract::collision::ContinuousContactManager::Ptr cm;
  mutex_.lock();
  auto it = continuous_contact_managers_.find(hash);
  if (it == continuous_contact_managers_.end())
  {
    cm = continuous_contact_manager_->clone();
    continuous_contact_managers_[hash] = cm;
  }
  else
  {
    cm = it->second;
  }
  mutex_.unlock();

  return tesseract::environment::checkTrajectory(results, *cm, *manip_, segment, collision_check_config_);
}

template <typename FloatType>
bool DescartesCollisionEdgeEvaluator<FloatType>::discreteCollisionCheck(
    std::vector<tesseract::collision::ContactResultMap>& results,
    const tesseract::common::TrajArray& segment) const
{
  // It was time using chronos time elapsed and it was faster to cache the contact manager
  unsigned long int hash = std::hash<std::thread::id>{}(std::this_thread::get_id());
  tesseract::collision::DiscreteContactManager::Ptr cm;
  mutex_.lock();
  auto it = discrete_contact_managers_.find(hash);
  if (it == discrete_contact_managers_.end())
  {
    cm = discrete_contact_manager_->clone();
    discrete_contact_managers_[hash] = cm;
  }
  else
  {
    cm = it->second;
  }
  mutex_.unlock();

  return tesseract::environment::checkTrajectory(results, *cm, *manip_, segment, collision_check_config_);
}

}  // namespace tesseract::motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_COLLISION_EDGE_EVALUATOR_HPP
