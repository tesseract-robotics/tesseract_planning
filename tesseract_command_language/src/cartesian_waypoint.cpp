#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/nvp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/eigen_serialization.h>
#include <tesseract_command_language/cartesian_waypoint.h>

template <class Archive>
void tesseract_planning::CartesianWaypoint::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(waypoint);
  ar& BOOST_SERIALIZATION_NVP(upper_tolerance);
  ar& BOOST_SERIALIZATION_NVP(lower_tolerance);
  ar& BOOST_SERIALIZATION_NVP(seed);
}

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::CartesianWaypoint)
TESSERACT_WAYPOINT_EXPORT_IMPLEMENT(tesseract_planning::CartesianWaypoint);
