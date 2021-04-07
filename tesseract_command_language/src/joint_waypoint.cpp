#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/vector.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/serialization.h>
#include <tesseract_command_language/joint_waypoint.h>

namespace tesseract_planning
{
void JointWaypoint::print(const std::string& prefix) const
{
  std::cout << prefix << "Joint WP: " << this->transpose() << std::endl;
}

bool JointWaypoint::isToleranced() const
{
  // Check if they are empty
  if (lower_tolerance.size() == 0 || upper_tolerance.size() == 0)
    return false;

  // Check if they are the same
  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  if ((lower_tolerance.array() > max_diff).any())
    throw std::runtime_error("JointWaypoint: lower tolerance was provided but must be <= 0,");

  if ((upper_tolerance.array() < -max_diff).any())
    throw std::runtime_error("JointWaypoint: upper tolerance was provided but must be >= 0,");

  return !tesseract_common::almostEqualRelativeAndAbs(lower_tolerance, upper_tolerance, max_diff);
}

bool JointWaypoint::operator==(const JointWaypoint& rhs) const
{
  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  bool equal = true;
  equal &= tesseract_common::almostEqualRelativeAndAbs(waypoint, rhs.waypoint, max_diff);
  equal &= tesseract_common::isIdentical(joint_names, rhs.joint_names);
  equal &= tesseract_common::almostEqualRelativeAndAbs(lower_tolerance, rhs.lower_tolerance, max_diff);
  equal &= tesseract_common::almostEqualRelativeAndAbs(upper_tolerance, rhs.upper_tolerance, max_diff);
  return equal;
}
bool JointWaypoint::operator!=(const JointWaypoint& rhs) const { return !operator==(rhs); }

template <class Archive>
void JointWaypoint::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(joint_names);
  ar& BOOST_SERIALIZATION_NVP(waypoint);
  ar& BOOST_SERIALIZATION_NVP(upper_tolerance);
  ar& BOOST_SERIALIZATION_NVP(lower_tolerance);
}
}  // namespace tesseract_planning

#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
template void tesseract_planning::JointWaypoint::serialize(boost::archive::xml_oarchive& ar,
                                                           const unsigned int version);
template void tesseract_planning::JointWaypoint::serialize(boost::archive::xml_iarchive& ar,
                                                           const unsigned int version);

TESSERACT_WAYPOINT_EXPORT_IMPLEMENT(tesseract_planning::JointWaypoint);
