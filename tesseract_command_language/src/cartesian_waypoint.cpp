#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/nvp.hpp>
#include <iostream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/serialization.h>
#include <tesseract_common/eigen_serialization.h>
#include <tesseract_common/utils.h>

#include <tesseract_command_language/cartesian_waypoint.h>

namespace tesseract_planning
{
CartesianWaypoint::CartesianWaypoint(const Eigen::Isometry3d& transform)  // NOLINT(modernize-pass-by-value)
  : transform_(transform)
{
}

CartesianWaypoint::CartesianWaypoint(const Eigen::Isometry3d& transform,  // NOLINT(modernize-pass-by-value)
                                     const Eigen::VectorXd& lower_tol,    // NOLINT(modernize-pass-by-value)
                                     const Eigen::VectorXd& upper_tol)    // NOLINT(modernize-pass-by-value)
  : transform_(transform), lower_tolerance_(lower_tol), upper_tolerance_(upper_tol)
{
}

// Waypoint
void CartesianWaypoint::setName(const std::string& name) { name_ = name; }
const std::string& CartesianWaypoint::getName() const { return name_; }
void CartesianWaypoint::print(const std::string& prefix) const
{
  std::cout << prefix << "Cart WP: xyz=" << transform_.translation().x() << ", " << transform_.translation().y()
            << ", "                                        // NOLINT
            << transform_.translation().z() << std::endl;  // NOLINT
  // TODO: Add rotation
}

std::unique_ptr<CartesianWaypointInterface> CartesianWaypoint::clone() const
{
  return std::make_unique<CartesianWaypoint>(*this);
}

// Cartesian Waypoint
void CartesianWaypoint::setTransform(const Eigen::Isometry3d& transform) { transform_ = transform; }
Eigen::Isometry3d& CartesianWaypoint::getTransform() { return transform_; }
const Eigen::Isometry3d& CartesianWaypoint::getTransform() const { return transform_; }

void CartesianWaypoint::setUpperTolerance(const Eigen::VectorXd& upper_tol) { upper_tolerance_ = upper_tol; }
Eigen::VectorXd& CartesianWaypoint::getUpperTolerance() { return upper_tolerance_; }
const Eigen::VectorXd& CartesianWaypoint::getUpperTolerance() const { return upper_tolerance_; }

void CartesianWaypoint::setLowerTolerance(const Eigen::VectorXd& lower_tol) { lower_tolerance_ = lower_tol; }
Eigen::VectorXd& CartesianWaypoint::getLowerTolerance() { return lower_tolerance_; }
const Eigen::VectorXd& CartesianWaypoint::getLowerTolerance() const { return lower_tolerance_; }

void CartesianWaypoint::setSeed(const tesseract_common::JointState& seed) { seed_ = seed; }
tesseract_common::JointState& CartesianWaypoint::getSeed() { return seed_; }
const tesseract_common::JointState& CartesianWaypoint::getSeed() const { return seed_; }

bool CartesianWaypoint::equals(const CartesianWaypointInterface& other) const
{
  const auto* rhs = dynamic_cast<const CartesianWaypoint*>(&other);
  if (rhs == nullptr)
    return false;

  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  bool equal = true;
  equal &= (name_ == rhs->name_);
  equal &= transform_.isApprox(rhs->transform_);
  equal &= tesseract_common::almostEqualRelativeAndAbs(lower_tolerance_, rhs->lower_tolerance_, max_diff);
  equal &= tesseract_common::almostEqualRelativeAndAbs(upper_tolerance_, rhs->upper_tolerance_, max_diff);
  equal &= (seed_ == rhs->seed_);
  return equal;
}

template <class Archive>
void CartesianWaypoint::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(CartesianWaypointInterface);
  ar& BOOST_SERIALIZATION_NVP(name_);
  ar& BOOST_SERIALIZATION_NVP(transform_);
  ar& BOOST_SERIALIZATION_NVP(upper_tolerance_);
  ar& BOOST_SERIALIZATION_NVP(lower_tolerance_);
  ar& BOOST_SERIALIZATION_NVP(seed_);
}
}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::CartesianWaypoint)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::CartesianWaypoint)
