/**
 * @file joint_waypoint_poly.h
 * @brief The joint waypoint interface
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
#ifndef TESSERACT_COMMAND_LANGUAGE_JOINT_WAYPOINT_POLY_H
#define TESSERACT_COMMAND_LANGUAGE_JOINT_WAYPOINT_POLY_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/concept_check.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/poly/waypoint_poly.h>
#include <tesseract_common/serialization.h>
#include <tesseract_common/type_erasure.h>

/** @brief If shared library, this must go in the header after the class definition */
#define TESSERACT_JOINT_WAYPOINT_EXPORT_KEY(N, C)                                                                      \
  namespace N                                                                                                          \
  {                                                                                                                    \
  using C##InstanceBase =                                                                                              \
      tesseract_common::TypeErasureInstance<C, tesseract_planning::detail_joint_waypoint::JointWaypointInterface>;     \
  using C##Instance = tesseract_planning::detail_joint_waypoint::JointWaypointInstance<C>;                             \
  using C##InstanceWrapper = tesseract_common::TypeErasureInstanceWrapper<C##Instance>;                                \
  }                                                                                                                    \
  BOOST_CLASS_EXPORT_KEY(N::C##InstanceBase)                                                                           \
  BOOST_CLASS_EXPORT_KEY(N::C##Instance)                                                                               \
  BOOST_CLASS_EXPORT_KEY(N::C##InstanceWrapper)                                                                        \
  BOOST_CLASS_TRACKING(N::C##InstanceBase, boost::serialization::track_never)                                          \
  BOOST_CLASS_TRACKING(N::C##Instance, boost::serialization::track_never)                                              \
  BOOST_CLASS_TRACKING(N::C##InstanceWrapper, boost::serialization::track_never)

/** @brief If shared library, this must go in the cpp after the implicit instantiation of the serialize function */
#define TESSERACT_JOINT_WAYPOINT_EXPORT_IMPLEMENT(inst)                                                                \
  BOOST_CLASS_EXPORT_IMPLEMENT(inst##InstanceBase)                                                                     \
  BOOST_CLASS_EXPORT_IMPLEMENT(inst##Instance)                                                                         \
  BOOST_CLASS_EXPORT_IMPLEMENT(inst##InstanceWrapper)

/**
 * @brief This should not be used within shared libraries use the two above.
 * If not in a shared library it can go in header or cpp
 */
#define TESSERACT_JOINT_WAYPOINT_EXPORT(N, C)                                                                          \
  TESSERACT_JOINT_WAYPOINT_EXPORT_KEY(N, C)                                                                            \
  TESSERACT_JOINT_WAYPOINT_EXPORT_IMPLEMENT(N::C)

namespace tesseract_planning::detail_joint_waypoint
{
template <typename T>
struct JointWaypointConcept  // NOLINT
  : boost::Assignable<T>,
    boost::CopyConstructible<T>,
    boost::EqualityComparable<T>
{
  BOOST_CONCEPT_USAGE(JointWaypointConcept)
  {
    T cp(c);
    T assign = c;
    bool eq = (c == cp);
    bool neq = (c != cp);
    UNUSED(assign);
    UNUSED(eq);
    UNUSED(neq);

    std::vector<std::string> names;
    c.setNames(names);

    std::vector<std::string>& names_ref = c.getNames();
    UNUSED(names_ref);

    const std::vector<std::string>& names_const_ref = c.getNames();
    UNUSED(names_const_ref);

    Eigen::VectorXd position;
    c.setPosition(position);

    Eigen::VectorXd& position_ref = c.getPosition();
    UNUSED(position_ref);

    const Eigen::VectorXd& position_const_ref = c.getPosition();
    UNUSED(position_const_ref);

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

    c.setIsConstrained(true);
    bool is_constrained = c.isConstrained();
    UNUSED(is_constrained);

    c.setName("name");
    c.getName();
    c.print();
    c.print("prefix_");
  }

private:
  T c;
};

struct JointWaypointInterface : tesseract_common::TypeErasureInterface
{
  virtual void setNames(const std::vector<std::string>& names) = 0;
  virtual std::vector<std::string>& getNames() = 0;
  virtual const std::vector<std::string>& getNames() const = 0;

  virtual void setPosition(const Eigen::VectorXd& position) = 0;
  virtual Eigen::VectorXd& getPosition() = 0;
  virtual const Eigen::VectorXd& getPosition() const = 0;

  virtual void setUpperTolerance(const Eigen::VectorXd& upper_tol) = 0;
  virtual Eigen::VectorXd& getUpperTolerance() = 0;
  virtual const Eigen::VectorXd& getUpperTolerance() const = 0;

  virtual void setLowerTolerance(const Eigen::VectorXd& lower_tol) = 0;
  virtual Eigen::VectorXd& getLowerTolerance() = 0;
  virtual const Eigen::VectorXd& getLowerTolerance() const = 0;

  virtual void setIsConstrained(bool value) = 0;
  virtual bool isConstrained() const = 0;

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
struct JointWaypointInstance : tesseract_common::TypeErasureInstance<T, JointWaypointInterface>  // NOLINT
{
  using BaseType = tesseract_common::TypeErasureInstance<T, JointWaypointInterface>;
  JointWaypointInstance() = default;
  JointWaypointInstance(const T& x) : BaseType(x) {}
  JointWaypointInstance(JointWaypointInstance&& x) noexcept : BaseType(std::move(x)) {}

  BOOST_CONCEPT_ASSERT((JointWaypointConcept<T>));

  void setNames(const std::vector<std::string>& names) final { this->get().setNames(names); }
  std::vector<std::string>& getNames() final { return this->get().getNames(); }
  const std::vector<std::string>& getNames() const final { return this->get().getNames(); }

  void setPosition(const Eigen::VectorXd& position) final { this->get().setPosition(position); }
  Eigen::VectorXd& getPosition() final { return this->get().getPosition(); }
  const Eigen::VectorXd& getPosition() const final { return this->get().getPosition(); }

  void setUpperTolerance(const Eigen::VectorXd& upper_tol) final { this->get().setUpperTolerance(upper_tol); }
  Eigen::VectorXd& getUpperTolerance() final { return this->get().getUpperTolerance(); }
  const Eigen::VectorXd& getUpperTolerance() const final { return this->get().getUpperTolerance(); }

  void setLowerTolerance(const Eigen::VectorXd& lower_tol) final { this->get().setLowerTolerance(lower_tol); }
  Eigen::VectorXd& getLowerTolerance() final { return this->get().getLowerTolerance(); }
  const Eigen::VectorXd& getLowerTolerance() const final { return this->get().getLowerTolerance(); }

  void setIsConstrained(bool value) final { this->get().setIsConstrained(value); }
  bool isConstrained() const final { return this->get().isConstrained(); }

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
}  // namespace tesseract_planning::detail_joint_waypoint

namespace tesseract_planning
{
using JointWaypointPolyBase = tesseract_common::TypeErasureBase<detail_joint_waypoint::JointWaypointInterface,
                                                                detail_joint_waypoint::JointWaypointInstance>;
struct JointWaypointPoly : JointWaypointPolyBase
{
  using JointWaypointPolyBase::JointWaypointPolyBase;

  void setNames(const std::vector<std::string>& names);
  std::vector<std::string>& getNames();
  const std::vector<std::string>& getNames() const;

  void setPosition(const Eigen::VectorXd& position);
  Eigen::VectorXd& getPosition();
  const Eigen::VectorXd& getPosition() const;

  void setUpperTolerance(const Eigen::VectorXd& upper_tol);
  Eigen::VectorXd& getUpperTolerance();
  const Eigen::VectorXd& getUpperTolerance() const;

  void setLowerTolerance(const Eigen::VectorXd& lower_tol);
  Eigen::VectorXd& getLowerTolerance();
  const Eigen::VectorXd& getLowerTolerance() const;

  void setIsConstrained(bool value);
  bool isConstrained() const;

  void setName(const std::string& name);
  const std::string& getName() const;

  void print(const std::string& prefix = "") const;

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

BOOST_CLASS_EXPORT_KEY(tesseract_planning::detail_joint_waypoint::JointWaypointInterface)
BOOST_CLASS_TRACKING(tesseract_planning::detail_joint_waypoint::JointWaypointInterface,
                     boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_planning::JointWaypointPolyBase)
BOOST_CLASS_TRACKING(tesseract_planning::JointWaypointPolyBase, boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_planning::JointWaypointPoly)
BOOST_CLASS_TRACKING(tesseract_planning::JointWaypointPoly, boost::serialization::track_never)

TESSERACT_WAYPOINT_EXPORT_KEY(tesseract_planning, JointWaypointPoly)

#endif  // TESSERACT_COMMAND_LANGUAGE_JOINT_WAYPOINT_POLY_H
