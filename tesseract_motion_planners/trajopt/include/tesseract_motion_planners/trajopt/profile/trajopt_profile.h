/**
 * @file trajopt_profile.h
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
#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajopt/fwd.hpp>
#include <trajopt_sco/optimizers.hpp>
#include <vector>
#include <memory>
#include <string>
#include <Eigen/Core>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/fwd.h>
#include <tesseract_environment/fwd.h>
#include <tesseract_command_language/fwd.h>
#include <tesseract_command_language/profile.h>

namespace boost::serialization
{
template <class Archive>
void serialize(Archive& ar, sco::BasicTrustRegionSQPParameters& params, const unsigned int version);  // NOLINT
}  // namespace boost::serialization

namespace tesseract_planning
{
/** @brief Structure to store TrajOpt cost and constrant term infos */
struct TrajOptTermInfos
{
  std::vector<std::shared_ptr<trajopt::TermInfo>> costs;
  std::vector<std::shared_ptr<trajopt::TermInfo>> constraints;
};

/** @brief Structure to store TrajOpt waypoint cost and constrant term infos */
struct TrajOptWaypointInfo
{
  TrajOptTermInfos term_infos;
  Eigen::VectorXd seed;
  bool fixed{ false };
};

class TrajOptPlanProfile : public Profile
{
public:
  using Ptr = std::shared_ptr<TrajOptPlanProfile>;
  using ConstPtr = std::shared_ptr<const TrajOptPlanProfile>;

  TrajOptPlanProfile();

  virtual TrajOptWaypointInfo create(const MoveInstructionPoly& move_instruction,
                                     const tesseract_common::ManipulatorInfo& composite_manip_info,
                                     const std::shared_ptr<const tesseract_environment::Environment>& env,
                                     const std::vector<std::string>& active_links,
                                     int index) const = 0;

  /**
   * @brief A utility function for getting profile ID
   * @return The profile ID used when storing in profile dictionary
   */
  static std::size_t getStaticKey();

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

class TrajOptCompositeProfile : public Profile
{
public:
  using Ptr = std::shared_ptr<TrajOptCompositeProfile>;
  using ConstPtr = std::shared_ptr<const TrajOptCompositeProfile>;

  TrajOptCompositeProfile();

  virtual TrajOptTermInfos create(const tesseract_common::ManipulatorInfo& composite_manip_info,
                                  const std::shared_ptr<const tesseract_environment::Environment>& env,
                                  const std::vector<int>& fixed_indices,
                                  int start_index,
                                  int end_index) const = 0;

  /**
   * @brief A utility function for getting profile ID
   * @return The profile ID used when storing in profile dictionary
   */
  static std::size_t getStaticKey();

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

class TrajOptSolverProfile : public Profile
{
public:
  using Ptr = std::shared_ptr<TrajOptSolverProfile>;
  using ConstPtr = std::shared_ptr<const TrajOptSolverProfile>;

  TrajOptSolverProfile();

  /** @brief Optimization paramters */
  sco::BasicTrustRegionSQPParameters opt_params;

  /** @brief Get the convex solver to use */
  virtual sco::ModelType getSolverType() const = 0;

  /** @brief The convex solver config to use, if nullptr the default settings are used */
  virtual std::unique_ptr<sco::ModelConfig> createSolverConfig() const = 0;

  /** @brief Optimization paramters */
  virtual sco::BasicTrustRegionSQPParameters createOptimizationParameters() const;

  /** @brief Optimization callbacks */
  virtual std::vector<sco::Optimizer::Callback> createOptimizationCallbacks() const;

  /**
   * @brief A utility function for getting profile ID
   * @return The profile ID used when storing in profile dictionary
   */
  static std::size_t getStaticKey();

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::TrajOptPlanProfile)
BOOST_CLASS_EXPORT_KEY(tesseract_planning::TrajOptCompositeProfile)
BOOST_CLASS_EXPORT_KEY(tesseract_planning::TrajOptSolverProfile)

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_PROFILE_H
