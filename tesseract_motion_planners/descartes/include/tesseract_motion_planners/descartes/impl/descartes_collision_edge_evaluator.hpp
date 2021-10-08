/**
 * @file descartes_collision_edge_evaluator.hpp
 * @brief Tesseract Descartes Collision Edge Evaluator Implementation
 *
 * @author Levi Armstrong
 * @date December 18, 2019
 * @version TODO
 * @bug No known bugs
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <numeric>
#include <thread>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/descartes/descartes_collision_edge_evaluator.h>
#include <tesseract_environment/utils.h>

namespace tesseract_planning
{
template <typename FloatType>
DescartesCollisionEdgeEvaluator<FloatType>::DescartesCollisionEdgeEvaluator(
    const tesseract_environment::Environment& collision_env,
    tesseract_kinematics::JointGroup::ConstPtr manip,
    tesseract_collision::CollisionCheckConfig config,
    bool allow_collision,
    bool debug)
  : manip_(std::move(manip))
  , acm_(*(collision_env.getAllowedCollisionMatrix()))
  , active_link_names_(manip_->getActiveLinkNames())
  , discrete_contact_manager_(collision_env.getDiscreteContactManager())
  , continuous_contact_manager_(collision_env.getContinuousContactManager())
  , collision_check_config_(std::move(config))
  , allow_collision_(allow_collision)
  , debug_(debug)
{
  discrete_contact_manager_->setActiveCollisionObjects(active_link_names_);
  discrete_contact_manager_->setCollisionMarginData(collision_check_config_.collision_margin_data,
                                                    collision_check_config_.collision_margin_override_type);
  discrete_contact_manager_->setIsContactAllowedFn(
      [this](const std::string& a, const std::string& b) { return isContactAllowed(a, b); });

  continuous_contact_manager_->setActiveCollisionObjects(active_link_names_);
  continuous_contact_manager_->setCollisionMarginData(collision_check_config_.collision_margin_data,
                                                      collision_check_config_.collision_margin_override_type);
  continuous_contact_manager_->setIsContactAllowedFn(
      [this](const std::string& a, const std::string& b) { return isContactAllowed(a, b); });
}

template <typename FloatType>
std::pair<bool, FloatType>
DescartesCollisionEdgeEvaluator<FloatType>::evaluate(const descartes_light::State<FloatType>& start,
                                                     const descartes_light::State<FloatType>& end) const
{
  assert(start.values.rows() == end.values.rows());

  // Happens in two phases:
  // 1. Compute the transform of all objects
  tesseract_common::TrajArray segment(2, start.values.rows());
  for (Eigen::Index i = 0; i < start.values.rows(); ++i)
  {
    segment(0, i) = start[i];
    segment(1, i) = end[i];
  }

  std::vector<tesseract_collision::ContactResultMap> discrete_results;
  std::vector<tesseract_collision::ContactResultMap> continuous_results;
  bool discrete_in_contact = discreteCollisionCheck(discrete_results, segment, allow_collision_);
  bool continuous_in_contact = continuousCollisionCheck(continuous_results, segment, allow_collision_);

  if (!discrete_in_contact && !continuous_in_contact)
    return std::make_pair(true, 0);

  // TODO: Update this to consider link pairs
  auto collision_safety_margin_ =
      static_cast<FloatType>(collision_check_config_.collision_margin_data.getMaxCollisionMargin());

  if (!discrete_in_contact && continuous_in_contact && allow_collision_)
    return std::make_pair(true, collision_safety_margin_ - continuous_results.begin()->begin()->second[0].distance);

  if (discrete_in_contact && !continuous_in_contact && allow_collision_)
    return std::make_pair(true, collision_safety_margin_ - discrete_results.begin()->begin()->second[0].distance);

  if (discrete_in_contact && continuous_in_contact && allow_collision_)
  {
    double d = collision_safety_margin_ - discrete_results.begin()->begin()->second[0].distance;
    double c = collision_safety_margin_ - continuous_results.begin()->begin()->second[0].distance;
    return std::make_pair(true, std::max(d, c));
  }

  return std::make_pair(false, 0);
}

template <typename FloatType>
bool DescartesCollisionEdgeEvaluator<FloatType>::isContactAllowed(const std::string& a, const std::string& b) const
{
  return acm_.isCollisionAllowed(a, b);
}

template <typename FloatType>
bool DescartesCollisionEdgeEvaluator<FloatType>::continuousCollisionCheck(
    std::vector<tesseract_collision::ContactResultMap>& results,
    const tesseract_common::TrajArray& segment,
    bool find_best) const
{
  // It was time using chronos time elapsed and it was faster to cache the contact manager
  unsigned long int hash = std::hash<std::thread::id>{}(std::this_thread::get_id());
  tesseract_collision::ContinuousContactManager::Ptr cm;
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
  tesseract_collision::CollisionCheckConfig config = collision_check_config_;
  if (config.type == tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE ||
      config.type == tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS)
    config.type = tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS;
  else
    config.type = tesseract_collision::CollisionEvaluatorType::CONTINUOUS;
  config.contact_request.type =
      (find_best) ? tesseract_collision::ContactTestType::CLOSEST : tesseract_collision::ContactTestType::FIRST;

  return tesseract_environment::checkTrajectory(results, *cm, *manip_, segment, config);
}

template <typename FloatType>
bool DescartesCollisionEdgeEvaluator<FloatType>::discreteCollisionCheck(
    std::vector<tesseract_collision::ContactResultMap>& results,
    const tesseract_common::TrajArray& segment,
    bool find_best) const
{
  // It was time using chronos time elapsed and it was faster to cache the contact manager
  unsigned long int hash = std::hash<std::thread::id>{}(std::this_thread::get_id());
  tesseract_collision::DiscreteContactManager::Ptr cm;
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

  tesseract_collision::CollisionCheckConfig config = collision_check_config_;
  if (config.type == tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE ||
      config.type == tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS)
    config.type = tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
  else
    config.type = tesseract_collision::CollisionEvaluatorType::DISCRETE;
  config.contact_request.type =
      (find_best) ? tesseract_collision::ContactTestType::CLOSEST : tesseract_collision::ContactTestType::FIRST;

  return tesseract_environment::checkTrajectory(results, *cm, *manip_, segment, config);
}

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_COLLISION_EDGE_EVALUATOR_HPP
