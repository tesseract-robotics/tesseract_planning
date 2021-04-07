#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/nvp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/serialization.h>
#include <tesseract_command_language/cartesian_waypoint.h>

template <class Archive>
void tesseract_planning::CartesianWaypoint::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(waypoint);
  ar& BOOST_SERIALIZATION_NVP(upper_tolerance);
  ar& BOOST_SERIALIZATION_NVP(lower_tolerance);
}

#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
template void tesseract_planning::CartesianWaypoint::serialize(boost::archive::xml_oarchive& ar,
                                                               const unsigned int version);
template void tesseract_planning::CartesianWaypoint::serialize(boost::archive::xml_iarchive& ar,
                                                               const unsigned int version);

TESSERACT_WAYPOINT_EXPORT_IMPLEMENT(tesseract_planning::CartesianWaypoint);
