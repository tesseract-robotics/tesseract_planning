/**
 * @file ompl_planner_configurator.h
 * @brief Tesseract OMPL planner configurator.
 *
 * If a settings class does not exist for a planner available
 * in ompl you may simply create your own that has an apply
 * method that takes the specific planner you would like to use
 * and construct the OMPLFreespacePlanner with the desired planner
 * and newly created config class and everything should work.
 *
 * @author Levi Armstrong
 * @date February 1, 2020
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
#ifndef TESSERACT_MOTION_PLANNERS_OMPL_PLANNER_CONFIGURATOR_H
#define TESSERACT_MOTION_PLANNERS_OMPL_PLANNER_CONFIGURATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <limits>
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace ompl::base
{
class Planner;
using PlannerPtr = std::shared_ptr<Planner>;
class SpaceInformation;
using SpaceInformationPtr = std::shared_ptr<SpaceInformation>;
}  // namespace ompl::base
namespace tesseract_planning
{
enum class OMPLPlannerType
{
  SBL = 0,
  EST = 1,
  LBKPIECE1 = 2,
  BKPIECE1 = 3,
  KPIECE1 = 4,
  BiTRRT = 5,
  RRT = 6,
  RRTConnect = 7,
  RRTstar = 8,
  TRRT = 9,
  PRM = 10,
  PRMstar = 11,
  LazyPRMstar = 12,
  SPARS = 13
};

struct OMPLPlannerConfigurator
{
  using Ptr = std::shared_ptr<OMPLPlannerConfigurator>;
  using ConstPtr = std::shared_ptr<const OMPLPlannerConfigurator>;

  OMPLPlannerConfigurator() = default;
  virtual ~OMPLPlannerConfigurator() = default;
  OMPLPlannerConfigurator(const OMPLPlannerConfigurator&) = default;
  OMPLPlannerConfigurator& operator=(const OMPLPlannerConfigurator&) = default;
  OMPLPlannerConfigurator(OMPLPlannerConfigurator&&) = default;
  OMPLPlannerConfigurator& operator=(OMPLPlannerConfigurator&&) = default;

  virtual ompl::base::PlannerPtr create(ompl::base::SpaceInformationPtr si) const = 0;

  virtual OMPLPlannerType getType() const = 0;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

struct SBLConfigurator : public OMPLPlannerConfigurator
{
  SBLConfigurator() = default;
  ~SBLConfigurator() override = default;
  SBLConfigurator(const SBLConfigurator&) = default;
  SBLConfigurator& operator=(const SBLConfigurator&) = default;
  SBLConfigurator(SBLConfigurator&&) = default;
  SBLConfigurator& operator=(SBLConfigurator&&) = default;

  /** @brief Max motion added to tree */
  double range = 0;

  /** @brief Create the planner */
  ompl::base::PlannerPtr create(ompl::base::SpaceInformationPtr si) const override;

  OMPLPlannerType getType() const override;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

struct ESTConfigurator : public OMPLPlannerConfigurator
{
  ESTConfigurator() = default;
  ~ESTConfigurator() override = default;
  ESTConfigurator(const ESTConfigurator&) = default;
  ESTConfigurator& operator=(const ESTConfigurator&) = default;
  ESTConfigurator(ESTConfigurator&&) = default;
  ESTConfigurator& operator=(ESTConfigurator&&) = default;

  /** @brief Max motion added to tree */
  double range = 0;

  /** @brief When close to goal select goal, with this probability. */
  double goal_bias = 0.05;

  /** @brief Create the planner */
  ompl::base::PlannerPtr create(ompl::base::SpaceInformationPtr si) const override;

  OMPLPlannerType getType() const override;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

struct LBKPIECE1Configurator : public OMPLPlannerConfigurator
{
  LBKPIECE1Configurator() = default;
  ~LBKPIECE1Configurator() override = default;
  LBKPIECE1Configurator(const LBKPIECE1Configurator&) = default;
  LBKPIECE1Configurator& operator=(const LBKPIECE1Configurator&) = default;
  LBKPIECE1Configurator(LBKPIECE1Configurator&&) = default;
  LBKPIECE1Configurator& operator=(LBKPIECE1Configurator&&) = default;

  /** @brief Max motion added to tree */
  double range = 0;

  /** @brief Fraction of time focused on boarder (0.0,1.] */
  double border_fraction = 0.9;

  /** @brief Accept partially valid moves above fraction. */
  double min_valid_path_fraction = 0.5;

  /** @brief Create the planner */
  ompl::base::PlannerPtr create(ompl::base::SpaceInformationPtr si) const override;

  OMPLPlannerType getType() const override;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

struct BKPIECE1Configurator : public OMPLPlannerConfigurator
{
  BKPIECE1Configurator() = default;
  ~BKPIECE1Configurator() override = default;
  BKPIECE1Configurator(const BKPIECE1Configurator&) = default;
  BKPIECE1Configurator& operator=(const BKPIECE1Configurator&) = default;
  BKPIECE1Configurator(BKPIECE1Configurator&&) = default;
  BKPIECE1Configurator& operator=(BKPIECE1Configurator&&) = default;

  /** @brief Max motion added to tree */
  double range = 0;

  /** @brief Fraction of time focused on boarder (0.0,1.] */
  double border_fraction = 0.9;

  /** @brief When extending motion fails, scale score by factor. */
  double failed_expansion_score_factor = 0.5;

  /** @brief Accept partially valid moves above fraction. */
  double min_valid_path_fraction = 0.5;

  /** @brief Create the planner */
  ompl::base::PlannerPtr create(ompl::base::SpaceInformationPtr si) const override;

  OMPLPlannerType getType() const override;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

struct KPIECE1Configurator : public OMPLPlannerConfigurator
{
  KPIECE1Configurator() = default;
  ~KPIECE1Configurator() override = default;
  KPIECE1Configurator(const KPIECE1Configurator&) = default;
  KPIECE1Configurator& operator=(const KPIECE1Configurator&) = default;
  KPIECE1Configurator(KPIECE1Configurator&&) = default;
  KPIECE1Configurator& operator=(KPIECE1Configurator&&) = default;

  /** @brief Max motion added to tree */
  double range = 0;

  /** @brief When close to goal select goal, with this probability. */
  double goal_bias = 0.05;

  /** @brief Fraction of time focused on boarder (0.0,1.] */
  double border_fraction = 0.9;

  /** @brief When extending motion fails, scale score by factor. */
  double failed_expansion_score_factor = 0.5;

  /** @brief Accept partially valid moves above fraction. */
  double min_valid_path_fraction = 0.5;

  /** @brief Create the planner */
  ompl::base::PlannerPtr create(ompl::base::SpaceInformationPtr si) const override;

  OMPLPlannerType getType() const override;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

struct BiTRRTConfigurator : public OMPLPlannerConfigurator
{
  BiTRRTConfigurator() = default;
  ~BiTRRTConfigurator() override = default;
  BiTRRTConfigurator(const BiTRRTConfigurator&) = default;
  BiTRRTConfigurator& operator=(const BiTRRTConfigurator&) = default;
  BiTRRTConfigurator(BiTRRTConfigurator&&) = default;
  BiTRRTConfigurator& operator=(BiTRRTConfigurator&&) = default;

  /** @brief Max motion added to tree */
  double range = 0;

  /** @brief How much to increase or decrease temp. */
  double temp_change_factor = 0.1;

  /** @brief Any motion cost that is not better than this cost (according to the optimization objective) will not be
   * expanded by the planner.  */
  double cost_threshold = std::numeric_limits<double>::infinity();

  /** @brief Initial temperature. */
  double init_temperature = 100.;

  /** @brief Dist new state to nearest neighbor to disqualify as frontier. */
  double frontier_threshold = 0.0;

  /** @brief 1/10, or 1 nonfrontier for every 10 frontier. */
  double frontier_node_ratio = 0.1;

  /** @brief Create the planner */
  ompl::base::PlannerPtr create(ompl::base::SpaceInformationPtr si) const override;

  OMPLPlannerType getType() const override;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

struct RRTConfigurator : public OMPLPlannerConfigurator
{
  RRTConfigurator() = default;
  ~RRTConfigurator() override = default;
  RRTConfigurator(const RRTConfigurator&) = default;
  RRTConfigurator& operator=(const RRTConfigurator&) = default;
  RRTConfigurator(RRTConfigurator&&) = default;
  RRTConfigurator& operator=(RRTConfigurator&&) = default;

  /** @brief Max motion added to tree */
  double range = 0;

  /** @brief When close to goal select goal, with this probability. */
  double goal_bias = 0.05;

  /** @brief Create the planner */
  ompl::base::PlannerPtr create(ompl::base::SpaceInformationPtr si) const override;

  OMPLPlannerType getType() const override;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

struct RRTConnectConfigurator : public OMPLPlannerConfigurator
{
  RRTConnectConfigurator() = default;
  ~RRTConnectConfigurator() override = default;
  RRTConnectConfigurator(const RRTConnectConfigurator&) = default;
  RRTConnectConfigurator& operator=(const RRTConnectConfigurator&) = default;
  RRTConnectConfigurator(RRTConnectConfigurator&&) = default;
  RRTConnectConfigurator& operator=(RRTConnectConfigurator&&) = default;

  /** @brief Max motion added to tree */
  double range = 0;

  /** @brief Create the planner */
  ompl::base::PlannerPtr create(ompl::base::SpaceInformationPtr si) const override;

  OMPLPlannerType getType() const override;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

struct RRTstarConfigurator : public OMPLPlannerConfigurator
{
  RRTstarConfigurator() = default;
  ~RRTstarConfigurator() override = default;
  RRTstarConfigurator(const RRTstarConfigurator&) = default;
  RRTstarConfigurator& operator=(const RRTstarConfigurator&) = default;
  RRTstarConfigurator(RRTstarConfigurator&&) = default;
  RRTstarConfigurator& operator=(RRTstarConfigurator&&) = default;

  /** @brief Max motion added to tree */
  double range = 0;

  /** @brief When close to goal select goal, with this probability. */
  double goal_bias = 0.05;

  /** @brief Stop collision checking as soon as C-free parent found. */
  bool delay_collision_checking = true;

  /** @brief Create the planner */
  ompl::base::PlannerPtr create(ompl::base::SpaceInformationPtr si) const override;

  OMPLPlannerType getType() const override;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

struct TRRTConfigurator : public OMPLPlannerConfigurator
{
  TRRTConfigurator() = default;
  ~TRRTConfigurator() override = default;
  TRRTConfigurator(const TRRTConfigurator&) = default;
  TRRTConfigurator& operator=(const TRRTConfigurator&) = default;
  TRRTConfigurator(TRRTConfigurator&&) = default;
  TRRTConfigurator& operator=(TRRTConfigurator&&) = default;

  /** @brief Max motion added to tree */
  double range = 0;

  /** @brief When close to goal select goal, with this probability. */
  double goal_bias = 0.05;

  /** @brief How much to increase or decrease temp. */
  double temp_change_factor = 2.0;

  /** @brief Initial temperature. */
  double init_temperature = 10e-6;

  /** @brief Dist new state to nearest neighbor to disqualify as frontier. */
  double frontier_threshold = 0.0;

  /** @brief 1/10, or 1 nonfrontier for every 10 frontier. */
  double frontier_node_ratio = 0.1;

  /** @brief Create the planner */
  ompl::base::PlannerPtr create(ompl::base::SpaceInformationPtr si) const override;

  OMPLPlannerType getType() const override;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

struct PRMConfigurator : public OMPLPlannerConfigurator
{
  PRMConfigurator() = default;
  ~PRMConfigurator() override = default;
  PRMConfigurator(const PRMConfigurator&) = default;
  PRMConfigurator& operator=(const PRMConfigurator&) = default;
  PRMConfigurator(PRMConfigurator&&) = default;
  PRMConfigurator& operator=(PRMConfigurator&&) = default;

  /** @brief Use k nearest neighbors. */
  int max_nearest_neighbors = 10;

  /** @brief Create the planner */
  ompl::base::PlannerPtr create(ompl::base::SpaceInformationPtr si) const override;

  OMPLPlannerType getType() const override;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

struct PRMstarConfigurator : public OMPLPlannerConfigurator
{
  PRMstarConfigurator() = default;
  ~PRMstarConfigurator() override = default;
  PRMstarConfigurator(const PRMstarConfigurator&) = default;
  PRMstarConfigurator& operator=(const PRMstarConfigurator&) = default;
  PRMstarConfigurator(PRMstarConfigurator&&) = default;
  PRMstarConfigurator& operator=(PRMstarConfigurator&&) = default;

  /** @brief Create the planner */
  ompl::base::PlannerPtr create(ompl::base::SpaceInformationPtr si) const override;

  OMPLPlannerType getType() const override;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

struct LazyPRMstarConfigurator : public OMPLPlannerConfigurator
{
  LazyPRMstarConfigurator() = default;
  ~LazyPRMstarConfigurator() override = default;
  LazyPRMstarConfigurator(const LazyPRMstarConfigurator&) = default;
  LazyPRMstarConfigurator& operator=(const LazyPRMstarConfigurator&) = default;
  LazyPRMstarConfigurator(LazyPRMstarConfigurator&&) = default;
  LazyPRMstarConfigurator& operator=(LazyPRMstarConfigurator&&) = default;

  /** @brief Create the planner */
  ompl::base::PlannerPtr create(ompl::base::SpaceInformationPtr si) const override;

  OMPLPlannerType getType() const override;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

struct SPARSConfigurator : public OMPLPlannerConfigurator
{
  SPARSConfigurator() = default;
  ~SPARSConfigurator() override = default;
  SPARSConfigurator(const SPARSConfigurator&) = default;
  SPARSConfigurator& operator=(const SPARSConfigurator&) = default;
  SPARSConfigurator(SPARSConfigurator&&) = default;
  SPARSConfigurator& operator=(SPARSConfigurator&&) = default;

  /** @brief The maximum number of failures before terminating the algorithm */
  int max_failures = 1000;

  /** @brief Dense graph connection distance as a fraction of max. extent */
  double dense_delta_fraction = 0.001;

  /** @brief Sparse Roadmap connection distance as a fraction of max. extent */
  double sparse_delta_fraction = 0.25;

  /** @brief The stretch factor in terms of graph spanners for SPARS to check against */
  double stretch_factor = 3;

  /** @brief Create the planner */
  ompl::base::PlannerPtr create(ompl::base::SpaceInformationPtr si) const override;

  OMPLPlannerType getType() const override;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::OMPLPlannerConfigurator)
BOOST_CLASS_EXPORT_KEY(tesseract_planning::SBLConfigurator)
BOOST_CLASS_EXPORT_KEY(tesseract_planning::ESTConfigurator)
BOOST_CLASS_EXPORT_KEY(tesseract_planning::LBKPIECE1Configurator)
BOOST_CLASS_EXPORT_KEY(tesseract_planning::BKPIECE1Configurator)
BOOST_CLASS_EXPORT_KEY(tesseract_planning::KPIECE1Configurator)
BOOST_CLASS_EXPORT_KEY(tesseract_planning::BiTRRTConfigurator)
BOOST_CLASS_EXPORT_KEY(tesseract_planning::RRTConfigurator)
BOOST_CLASS_EXPORT_KEY(tesseract_planning::RRTConnectConfigurator)
BOOST_CLASS_EXPORT_KEY(tesseract_planning::RRTstarConfigurator)
BOOST_CLASS_EXPORT_KEY(tesseract_planning::TRRTConfigurator)
BOOST_CLASS_EXPORT_KEY(tesseract_planning::PRMConfigurator)
BOOST_CLASS_EXPORT_KEY(tesseract_planning::PRMstarConfigurator)
BOOST_CLASS_EXPORT_KEY(tesseract_planning::LazyPRMstarConfigurator)
BOOST_CLASS_EXPORT_KEY(tesseract_planning::SPARSConfigurator)

#endif  // TESSERACT_MOTION_PLANNERS_OMPL_PLANNER_CONFIGURATOR_H
