#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/nvp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/serialization.h>
#include <tesseract_command_language/core/waypoint.h>

template <class Archive>
void tesseract_planning::detail_waypoint::WaypointInterface::serialize(Archive& ar,
                                                                       const unsigned int /*version*/)  // NOLINT
{
  ar& boost::serialization::make_nvp("base",
                                     boost::serialization::base_object<tesseract_common::TypeErasureInterface>(*this));
}

void tesseract_planning::Waypoint::print(const std::string& prefix) const { getInterface().print(prefix); }

template <class Archive>
void tesseract_planning::Waypoint::serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
{
  ar& boost::serialization::make_nvp("base", boost::serialization::base_object<WaypointBase>(*this));
}

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::detail_waypoint::WaypointInterface)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::WaypointBase)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::Waypoint)

BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::detail_waypoint::WaypointInterface)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::WaypointBase)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::Waypoint)
