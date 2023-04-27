/**
 * @file cartesian_waypoint_poly.h
 * @brief The cartesian waypoint interface
 *
 * @author Levi Armstrong
 * @date June 15, 2020
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
#ifndef TESSERACT_COMMAND_LANGUAGE_CARTESIAN_WAYPOINT_POLY_H
#define TESSERACT_COMMAND_LANGUAGE_CARTESIAN_WAYPOINT_POLY_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/concept_check.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/poly/waypoint_poly.h>
#include <tesseract_common/joint_state.h>
#include <tesseract_common/serialization.h>
#include <tesseract_common/type_erasure.h>

/** @brief If shared library, this must go in the header after the class definition */
#define TESSERACT_CARTESIAN_WAYPOINT_EXPORT_KEY(N, C)                                                                  \
  namespace N                                                                                                          \
  {                                                                                                                    \
  using C##InstanceBase = tesseract_common::                                                                           \
      TypeErasureInstance<C, tesseract_planning::detail_cartesian_waypoint::CartesianWaypointInterface>;               \
  using C##Instance = tesseract_planning::detail_cartesian_waypoint::CartesianWaypointInstance<C>;                     \
  using C##InstanceWrapper = tesseract_common::TypeErasureInstanceWrapper<C##Instance>;                                \
  }                                                                                                                    \
  BOOST_CLASS_EXPORT_KEY(N::C##InstanceBase)                                                                           \
  BOOST_CLASS_EXPORT_KEY(N::C##Instance)                                                                               \
  BOOST_CLASS_EXPORT_KEY(N::C##InstanceWrapper)                                                                        \
  BOOST_CLASS_TRACKING(N::C##InstanceBase, boost::serialization::track_never)                                          \
  BOOST_CLASS_TRACKING(N::C##Instance, boost::serialization::track_never)                                              \
  BOOST_CLASS_TRACKING(N::C##InstanceWrapper, boost::serialization::track_never)

/** @brief If shared library, this must go in the cpp after the implicit instantiation of the serialize function */
#define TESSERACT_CARTESIAN_WAYPOINT_EXPORT_IMPLEMENT(inst)                                                            \
  BOOST_CLASS_EXPORT_IMPLEMENT(inst##InstanceBase)                                                                     \
  BOOST_CLASS_EXPORT_IMPLEMENT(inst##Instance)                                                                         \
  BOOST_CLASS_EXPORT_IMPLEMENT(inst##InstanceWrapper)

/**
 * @brief This should not be used within shared libraries use the two above.
 * If not in a shared library it can go in header or cpp
 */
#define TESSERACT_CARTESIAN_WAYPOINT_EXPORT(N, C)                                                                      \
  TESSERACT_CARTESIAN_WAYPOINT_EXPORT_KEY(N, C)                                                                        \
  TESSERACT_CARTESIAN_WAYPOINT_EXPORT_IMPLEMENT(N::C)
namespace tesseract_planning::detail_cartesian_waypoint
{
template <typename T>
struct CartesianWaypointConcept  // NOLINT
  : boost::Assignable<T>,
    boost::CopyConstructible<T>,
    boost::EqualityComparable<T>
{
  BOOST_CONCEPT_USAGE(CartesianWaypointConcept)
  {
    T cp(c);
    T assign = c;
    bool eq = (c == cp);
    bool neq = (c != cp);
    UNUSED(assign);
    UNUSED(eq);
    UNUSED(neq);

    Eigen::Isometry3d transform;
    c.setTransform(transform);

    Eigen::Isometry3d& transform_ref = c.getTransform();
    UNUSED(transform_ref);

    const Eigen::Isometry3d& transform_const_ref = c.getTransform();
    UNUSED(transform_const_ref);

    Eigen::VectorXd lower_tol;
    c.setLowerTolerance(lower_tol);

    Eigen::VectorXd& lower_tol_ref = c.getLowerTolerance();
    UNUSED(lower_tol_ref);

    const Eigen::VectorXd& lower_tol_const_ref = c.getLowerTolerance();
    UNUSED(lower_tol_const_ref);

    Eigen::VectorXd upper_tol;
    c.setUpperTolerance(upper_tol);

    Eigen::VectorXd& upper_tol_ref = c.getUpperTolerance();
    UNUSED(upper_tol_ref);

    const Eigen::VectorXd& upper_tol_const_ref = c.getUpperTolerance();
    UNUSED(upper_tol_const_ref);

    tesseract_common::JointState js;
    c.setSeed(js);

    tesseract_common::JointState& seed_ref = c.getSeed();
    UNUSED(seed_ref);

    const tesseract_common::JointState& seed_const_ref = c.getSeed();
    UNUSED(seed_const_ref);

    c.setName("name");
    c.getName();
    c.print();
    c.print("prefix_");
  }

private:
  T c;
};

struct CartesianWaypointInterface : tesseract_common::TypeErasureInterface
{
  virtual void setTransform(const Eigen::Isometry3d& transform) = 0;
  virtual Eigen::Isometry3d& getTransform() = 0;
  virtual const Eigen::Isometry3d& getTransform() const = 0;

  virtual void setUpperTolerance(const Eigen::VectorXd& upper_tol) = 0;
  virtual Eigen::VectorXd& getUpperTolerance() = 0;
  virtual const Eigen::VectorXd& getUpperTolerance() const = 0;

  virtual void setLowerTolerance(const Eigen::VectorXd& lower_tol) = 0;
  virtual Eigen::VectorXd& getLowerTolerance() = 0;
  virtual const Eigen::VectorXd& getLowerTolerance() const = 0;

  virtual void setSeed(const tesseract_common::JointState& seed) = 0;
  virtual tesseract_common::JointState& getSeed() = 0;
  virtual const tesseract_common::JointState& getSeed() const = 0;

  virtual void setName(const std::string& name) = 0;
  virtual const std::string& getName() const = 0;

  virtual void print(const std::string& prefix) const = 0;

private:
  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

template <typename T>
struct CartesianWaypointInstance : tesseract_common::TypeErasureInstance<T, CartesianWaypointInterface>  // NOLINT
{
  using BaseType = tesseract_common::TypeErasureInstance<T, CartesianWaypointInterface>;
  CartesianWaypointInstance() = default;
  CartesianWaypointInstance(const T& x) : BaseType(x) {}
  CartesianWaypointInstance(CartesianWaypointInstance&& x) noexcept : BaseType(std::move(x)) {}

  BOOST_CONCEPT_ASSERT((CartesianWaypointConcept<T>));

  void setTransform(const Eigen::Isometry3d& transform) final { this->get().setTransform(transform); }
  Eigen::Isometry3d& getTransform() final { return this->get().getTransform(); }
  const Eigen::Isometry3d& getTransform() const final { return this->get().getTransform(); }

  void setUpperTolerance(const Eigen::VectorXd& upper_tol) final { this->get().setUpperTolerance(upper_tol); }
  Eigen::VectorXd& getUpperTolerance() final { return this->get().getUpperTolerance(); }
  const Eigen::VectorXd& getUpperTolerance() const final { return this->get().getUpperTolerance(); }

  void setLowerTolerance(const Eigen::VectorXd& lower_tol) final { this->get().setLowerTolerance(lower_tol); }
  Eigen::VectorXd& getLowerTolerance() final { return this->get().getLowerTolerance(); }
  const Eigen::VectorXd& getLowerTolerance() const final { return this->get().getLowerTolerance(); }

  void setSeed(const tesseract_common::JointState& seed) final { this->get().setSeed(seed); }
  tesseract_common::JointState& getSeed() final { return this->get().getSeed(); }
  const tesseract_common::JointState& getSeed() const final { return this->get().getSeed(); }

  void setName(const std::string& name) final { this->get().setName(name); }
  const std::string& getName() const final { return this->get().getName(); }
  void print(const std::string& prefix) const final { this->get().print(prefix); }

private:
  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
  {
    ar& boost::serialization::make_nvp("base", boost::serialization::base_object<BaseType>(*this));
  }
};
}  // namespace tesseract_planning::detail_cartesian_waypoint
namespace tesseract_planning
{
using CartesianWaypointPolyBase =
    tesseract_common::TypeErasureBase<detail_cartesian_waypoint::CartesianWaypointInterface,
                                      detail_cartesian_waypoint::CartesianWaypointInstance>;
struct CartesianWaypointPoly : CartesianWaypointPolyBase
{
  using CartesianWaypointPolyBase::CartesianWaypointPolyBase;

  void setTransform(const Eigen::Isometry3d& transform);
  Eigen::Isometry3d& getTransform();
  const Eigen::Isometry3d& getTransform() const;

  void setUpperTolerance(const Eigen::VectorXd& upper_tol);
  Eigen::VectorXd& getUpperTolerance();
  const Eigen::VectorXd& getUpperTolerance() const;

  void setLowerTolerance(const Eigen::VectorXd& lower_tol);
  Eigen::VectorXd& getLowerTolerance();
  const Eigen::VectorXd& getLowerTolerance() const;

  void setSeed(const tesseract_common::JointState& seed);
  tesseract_common::JointState& getSeed();
  const tesseract_common::JointState& getSeed() const;

  void setName(const std::string& name);
  const std::string& getName() const;

  void print(const std::string& prefix = "") const;

  /**
   * @brief Check if it has a seed. If the position or joint names is empty this returns false
   * @return True if it has a seed, otherwise false
   */
  bool hasSeed() const;

  /** @brief Clear the seed to empty data structures */
  void clearSeed();

  /**
   * @brief Returns true if waypoint has tolerances
   * @return True if waypoint has tolerances
   */
  bool isToleranced() const;

private:
  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/);  // NOLINT
};

}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::detail_cartesian_waypoint::CartesianWaypointInterface)
BOOST_CLASS_TRACKING(tesseract_planning::detail_cartesian_waypoint::CartesianWaypointInterface,
                     boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_planning::CartesianWaypointPolyBase)
BOOST_CLASS_TRACKING(tesseract_planning::CartesianWaypointPolyBase, boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_planning::CartesianWaypointPoly)
BOOST_CLASS_TRACKING(tesseract_planning::CartesianWaypointPoly, boost::serialization::track_never);

TESSERACT_WAYPOINT_EXPORT_KEY(tesseract_planning, CartesianWaypointPoly);

#endif  // TESSERACT_COMMAND_LANGUAGE_CARTESIAN_WAYPOINT_POLY_H
