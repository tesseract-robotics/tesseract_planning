#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <iostream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/utils.h>
#include <tesseract/common/types.h>
#include <tesseract/command_language/joint_waypoint.h>

namespace tesseract::command_language
{
// NOLINTNEXTLINE(modernize-pass-by-value)
JointWaypoint::JointWaypoint(std::vector<std::string> names, const Eigen::VectorXd& position, bool is_constrained)
  : position_(position), is_constrained_(is_constrained)
{
  joint_ids_.reserve(names.size());
  for (auto& name : names)
    joint_ids_.push_back(tesseract::common::JointId::fromName(name));

  if (static_cast<Eigen::Index>(joint_ids_.size()) != position_.size())
    throw std::runtime_error("JointWaypoint: parameters are not the same size!");
}

JointWaypoint::JointWaypoint(std::vector<std::string> names,
                             const Eigen::VectorXd& position,   // NOLINT(modernize-pass-by-value)
                             const Eigen::VectorXd& lower_tol,  // NOLINT(modernize-pass-by-value)
                             const Eigen::VectorXd& upper_tol)  // NOLINT(modernize-pass-by-value)
  : position_(position), lower_tolerance_(lower_tol), upper_tolerance_(upper_tol), is_constrained_(true)
{
  joint_ids_.reserve(names.size());
  for (auto& name : names)
    joint_ids_.push_back(tesseract::common::JointId::fromName(name));

  if (static_cast<Eigen::Index>(joint_ids_.size()) != position_.size() ||
      position_.size() != lower_tolerance_.size() || position_.size() != upper_tolerance_.size())
    throw std::runtime_error("JointWaypoint: parameters are not the same size!");
}

JointWaypoint::JointWaypoint(std::initializer_list<std::string> names,
                             std::initializer_list<double> position,
                             bool is_constrained)
  : JointWaypoint(names,
                  Eigen::Map<const Eigen::VectorXd>(position.begin(), static_cast<Eigen::Index>(position.size())),
                  is_constrained)
{
}

JointWaypoint::JointWaypoint(std::initializer_list<std::string> names,
                             std::initializer_list<double> position,
                             std::initializer_list<double> lower_tol,
                             std::initializer_list<double> upper_tol)
  : JointWaypoint(names,
                  Eigen::Map<const Eigen::VectorXd>(position.begin(), static_cast<Eigen::Index>(position.size())),
                  Eigen::Map<const Eigen::VectorXd>(lower_tol.begin(), static_cast<Eigen::Index>(lower_tol.size())),
                  Eigen::Map<const Eigen::VectorXd>(upper_tol.begin(), static_cast<Eigen::Index>(upper_tol.size())))
{
}

// Waypoint
void JointWaypoint::setName(const std::string& name) { name_ = name; }
const std::string& JointWaypoint::getName() const { return name_; }
void JointWaypoint::print(const std::string& prefix) const
{
  std::cout << prefix << "Joint WP: " << position_.transpose() << std::endl;  // NOLINT
}

std::unique_ptr<JointWaypointInterface> JointWaypoint::clone() const { return std::make_unique<JointWaypoint>(*this); }

// Joint Waypoint
void JointWaypoint::setNames(const std::vector<std::string>& names)
{
  joint_ids_.clear();
  joint_ids_.reserve(names.size());
  for (const auto& name : names)
    joint_ids_.push_back(tesseract::common::JointId::fromName(name));
}

std::vector<std::string> JointWaypoint::getNames() const
{
  std::vector<std::string> names;
  names.reserve(joint_ids_.size());
  for (const auto& id : joint_ids_)
    names.push_back(id.name());
  return names;
}

void JointWaypoint::setJointIds(const std::vector<tesseract::common::JointId>& ids) { joint_ids_ = ids; }
const std::vector<tesseract::common::JointId>& JointWaypoint::getJointIds() const { return joint_ids_; }
std::vector<tesseract::common::JointId>& JointWaypoint::getJointIds() { return joint_ids_; }

void JointWaypoint::setPosition(const Eigen::VectorXd& position) { position_ = position; }
Eigen::VectorXd& JointWaypoint::getPosition() { return position_; }
const Eigen::VectorXd& JointWaypoint::getPosition() const { return position_; }

void JointWaypoint::setUpperTolerance(const Eigen::VectorXd& upper_tol) { upper_tolerance_ = upper_tol; }
Eigen::VectorXd& JointWaypoint::getUpperTolerance() { return upper_tolerance_; }
const Eigen::VectorXd& JointWaypoint::getUpperTolerance() const { return upper_tolerance_; }

void JointWaypoint::setLowerTolerance(const Eigen::VectorXd& lower_tol) { lower_tolerance_ = lower_tol; }
Eigen::VectorXd& JointWaypoint::getLowerTolerance() { return lower_tolerance_; }
const Eigen::VectorXd& JointWaypoint::getLowerTolerance() const { return lower_tolerance_; }

void JointWaypoint::setIsConstrained(bool value) { is_constrained_ = value; }
bool JointWaypoint::isConstrained() const { return is_constrained_; }

bool JointWaypoint::equals(const JointWaypointInterface& other) const
{
  const auto* rhs = dynamic_cast<const JointWaypoint*>(&other);
  if (rhs == nullptr)
    return false;

  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  bool equal = true;
  equal &= (name_ == rhs->name_);
  equal &= (joint_ids_ == rhs->joint_ids_);
  equal &= tesseract::common::almostEqualRelativeAndAbs(position_, rhs->position_, max_diff);
  equal &= tesseract::common::almostEqualRelativeAndAbs(lower_tolerance_, rhs->lower_tolerance_, max_diff);
  equal &= tesseract::common::almostEqualRelativeAndAbs(upper_tolerance_, rhs->upper_tolerance_, max_diff);
  equal &= (is_constrained_ == rhs->is_constrained_);
  return equal;
}

}  // namespace tesseract::command_language
