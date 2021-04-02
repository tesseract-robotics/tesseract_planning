#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/export.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/vector.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/serialization.h>
#include <tesseract_command_language/joint_waypoint.h>

template <class Archive>
void tesseract_planning::JointWaypoint::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(joint_names);
  ar& BOOST_SERIALIZATION_NVP(waypoint);
  ar& BOOST_SERIALIZATION_NVP(upper_tolerance);
  ar& BOOST_SERIALIZATION_NVP(lower_tolerance);
}

TESSERACT_WAYPOINT_IMPLEMENT(tesseract_planning::JointWaypoint);

#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
template void tesseract_planning::JointWaypoint::serialize(boost::archive::xml_oarchive& ar,
                                                           const unsigned int version);
template void tesseract_planning::JointWaypoint::serialize(boost::archive::xml_iarchive& ar,
                                                           const unsigned int version);
