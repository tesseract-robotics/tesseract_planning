/**
 * @file state_waypoint_poly.h
 * @brief The state waypoint interface
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
#ifndef TESSERACT_COMMAND_LANGUAGE_STATE_WAYPOINT_POLY_H
#define TESSERACT_COMMAND_LANGUAGE_STATE_WAYPOINT_POLY_H
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
#define TESSERACT_STATE_WAYPOINT_EXPORT_KEY(N, C)                                                                      \
  namespace N                                                                                                          \
  {                                                                                                                    \
  using C##InstanceBase =                                                                                              \
      tesseract_common::TypeErasureInstance<C, tesseract_planning::detail_state_waypoint::StateWaypointInterface>;     \
  using C##Instance = tesseract_planning::detail_state_waypoint::StateWaypointInstance<C>;                             \
  using C##InstanceWrapper = tesseract_common::TypeErasureInstanceWrapper<C##Instance>;                                \
  }                                                                                                                    \
  BOOST_CLASS_EXPORT_KEY(N::C##InstanceBase)                                                                           \
  BOOST_CLASS_EXPORT_KEY(N::C##Instance)                                                                               \
  BOOST_CLASS_EXPORT_KEY(N::C##InstanceWrapper)                                                                        \
  BOOST_CLASS_TRACKING(N::C##InstanceBase, boost::serialization::track_never)                                          \
  BOOST_CLASS_TRACKING(N::C##Instance, boost::serialization::track_never)                                              \
  BOOST_CLASS_TRACKING(N::C##InstanceWrapper, boost::serialization::track_never)

/** @brief If shared library, this must go in the cpp after the implicit instantiation of the serialize function */
#define TESSERACT_STATE_WAYPOINT_EXPORT_IMPLEMENT(inst)                                                                \
  BOOST_CLASS_EXPORT_IMPLEMENT(inst##InstanceBase)                                                                     \
  BOOST_CLASS_EXPORT_IMPLEMENT(inst##Instance)                                                                         \
  BOOST_CLASS_EXPORT_IMPLEMENT(inst##InstanceWrapper)

/**
 * @brief This should not be used within shared libraries use the two above.
 * If not in a shared library it can go in header or cpp
 */
#define TESSERACT_STATE_WAYPOINT_EXPORT(N, C)                                                                          \
  TESSERACT_JOINT_WAYPOINT_EXPORT_KEY(N, C)                                                                            \
  TESSERACT_JOINT_WAYPOINT_EXPORT_IMPLEMENT(N::C)

namespace tesseract_planning::detail_state_waypoint
{
template <typename T>
struct StateWaypointConcept  // NOLINT
  : boost::Assignable<T>,
    boost::CopyConstructible<T>,
    boost::EqualityComparable<T>
{
  BOOST_CONCEPT_USAGE(StateWaypointConcept)
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

    Eigen::VectorXd velocity;
    c.setVelocity(velocity);

    Eigen::VectorXd& velocity_ref = c.getVelocity();
    UNUSED(velocity_ref);

    const Eigen::VectorXd& velocity_const_ref = c.getVelocity();
    UNUSED(velocity_const_ref);

    Eigen::VectorXd acceleration;
    c.setAcceleration(acceleration);

    Eigen::VectorXd& acceleration_ref = c.getAcceleration();
    UNUSED(acceleration_ref);

    const Eigen::VectorXd& acceleration_const_ref = c.getAcceleration();
    UNUSED(acceleration_const_ref);

    Eigen::VectorXd effort;
    c.setEffort(effort);

    Eigen::VectorXd& effort_ref = c.getEffort();
    UNUSED(effort_ref);

    const Eigen::VectorXd& effort_const_ref = c.getEffort();
    UNUSED(effort_const_ref);

    double time{ 0 };
    c.setTime(time);

    double time_copy = c.getTime();
    UNUSED(time_copy);

    c.setName("name");
    c.getName();
    c.print();
    c.print("prefix_");
  }

private:
  T c;
};

struct StateWaypointInterface : tesseract_common::TypeErasureInterface
{
  virtual void setNames(const std::vector<std::string>& names) = 0;
  virtual std::vector<std::string>& getNames() = 0;
  virtual const std::vector<std::string>& getNames() const = 0;

  virtual void setPosition(const Eigen::VectorXd& position) = 0;
  virtual Eigen::VectorXd& getPosition() = 0;
  virtual const Eigen::VectorXd& getPosition() const = 0;

  virtual void setVelocity(const Eigen::VectorXd& velocity) = 0;
  virtual Eigen::VectorXd& getVelocity() = 0;
  virtual const Eigen::VectorXd& getVelocity() const = 0;

  virtual void setAcceleration(const Eigen::VectorXd& acceleration) = 0;
  virtual Eigen::VectorXd& getAcceleration() = 0;
  virtual const Eigen::VectorXd& getAcceleration() const = 0;

  virtual void setEffort(const Eigen::VectorXd& effort) = 0;
  virtual Eigen::VectorXd& getEffort() = 0;
  virtual const Eigen::VectorXd& getEffort() const = 0;

  virtual void setTime(double time) = 0;
  virtual double getTime() const = 0;

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
struct StateWaypointInstance : tesseract_common::TypeErasureInstance<T, StateWaypointInterface>  // NOLINT
{
  using BaseType = tesseract_common::TypeErasureInstance<T, StateWaypointInterface>;
  StateWaypointInstance() = default;
  StateWaypointInstance(const T& x) : BaseType(x) {}
  StateWaypointInstance(StateWaypointInstance&& x) noexcept : BaseType(std::move(x)) {}

  BOOST_CONCEPT_ASSERT((StateWaypointConcept<T>));

  void setNames(const std::vector<std::string>& names) final { this->get().setNames(names); }
  std::vector<std::string>& getNames() final { return this->get().getNames(); }
  const std::vector<std::string>& getNames() const final { return this->get().getNames(); }

  void setPosition(const Eigen::VectorXd& position) final { this->get().setPosition(position); }
  Eigen::VectorXd& getPosition() final { return this->get().getPosition(); }
  const Eigen::VectorXd& getPosition() const final { return this->get().getPosition(); }

  void setVelocity(const Eigen::VectorXd& velocity) final { this->get().setVelocity(velocity); }
  Eigen::VectorXd& getVelocity() final { return this->get().getVelocity(); }
  const Eigen::VectorXd& getVelocity() const final { return this->get().getVelocity(); }

  void setAcceleration(const Eigen::VectorXd& acceleration) final { this->get().setAcceleration(acceleration); }
  Eigen::VectorXd& getAcceleration() final { return this->get().getAcceleration(); }
  const Eigen::VectorXd& getAcceleration() const final { return this->get().getAcceleration(); }

  void setEffort(const Eigen::VectorXd& effort) final { this->get().setEffort(effort); }
  Eigen::VectorXd& getEffort() final { return this->get().getEffort(); }
  const Eigen::VectorXd& getEffort() const final { return this->get().getEffort(); }

  void setTime(double time) final { this->get().setTime(time); }
  double getTime() const final { return this->get().getTime(); }

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
}  // namespace tesseract_planning::detail_state_waypoint
namespace tesseract_planning
{
using StateWaypointPolyBase = tesseract_common::TypeErasureBase<detail_state_waypoint::StateWaypointInterface,
                                                                detail_state_waypoint::StateWaypointInstance>;
struct StateWaypointPoly : StateWaypointPolyBase
{
  using StateWaypointPolyBase::StateWaypointPolyBase;

  void setNames(const std::vector<std::string>& names);
  std::vector<std::string>& getNames();
  const std::vector<std::string>& getNames() const;

  void setPosition(const Eigen::VectorXd& position);
  Eigen::VectorXd& getPosition();
  const Eigen::VectorXd& getPosition() const;

  void setVelocity(const Eigen::VectorXd& velocity);
  Eigen::VectorXd& getVelocity();
  const Eigen::VectorXd& getVelocity() const;

  void setAcceleration(const Eigen::VectorXd& acceleration);
  Eigen::VectorXd& getAcceleration();
  const Eigen::VectorXd& getAcceleration() const;

  void setEffort(const Eigen::VectorXd& effort);
  Eigen::VectorXd& getEffort();
  const Eigen::VectorXd& getEffort() const;

  void setTime(double time);
  double getTime() const;

  void setName(const std::string& name);
  const std::string& getName() const;

  void print(const std::string& prefix = "") const;

private:
  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/);  // NOLINT
};

}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::detail_state_waypoint::StateWaypointInterface)
BOOST_CLASS_TRACKING(tesseract_planning::detail_state_waypoint::StateWaypointInterface,
                     boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_planning::StateWaypointPolyBase)
BOOST_CLASS_TRACKING(tesseract_planning::StateWaypointPolyBase, boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_planning::StateWaypointPoly)
BOOST_CLASS_TRACKING(tesseract_planning::StateWaypointPoly, boost::serialization::track_never);

TESSERACT_WAYPOINT_EXPORT_KEY(tesseract_planning, StateWaypointPoly);

#endif  // TESSERACT_COMMAND_LANGUAGE_STATE_WAYPOINT_POLY_H
