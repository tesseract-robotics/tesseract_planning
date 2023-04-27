#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/nvp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/poly/waypoint_poly.h>
#include <tesseract_command_language/poly/cartesian_waypoint_poly.h>
#include <tesseract_command_language/poly/joint_waypoint_poly.h>
#include <tesseract_command_language/poly/state_waypoint_poly.h>

template <class Archive>
void tesseract_planning::detail_waypoint::WaypointInterface::serialize(Archive& ar,
                                                                       const unsigned int /*version*/)  // NOLINT
{
  ar& boost::serialization::make_nvp("base",
                                     boost::serialization::base_object<tesseract_common::TypeErasureInterface>(*this));
}

void tesseract_planning::WaypointPoly::setName(const std::string& name) { getInterface().setName(name); }
const std::string& tesseract_planning::WaypointPoly::getName() const { return getInterface().getName(); }

void tesseract_planning::WaypointPoly::print(const std::string& prefix) const { getInterface().print(prefix); }

bool tesseract_planning::WaypointPoly::isCartesianWaypoint() const
{
  return (isNull() ? false : (getInterface().getType() == std::type_index(typeid(CartesianWaypointPoly))));
}

bool tesseract_planning::WaypointPoly::isJointWaypoint() const
{
  return (isNull() ? false : (getInterface().getType() == std::type_index(typeid(JointWaypointPoly))));
}

bool tesseract_planning::WaypointPoly::isStateWaypoint() const
{
  return (isNull() ? false : (getInterface().getType() == std::type_index(typeid(StateWaypointPoly))));
}

template <class Archive>
void tesseract_planning::WaypointPoly::serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
{
  ar& boost::serialization::make_nvp("base", boost::serialization::base_object<WaypointPolyBase>(*this));
}

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::detail_waypoint::WaypointInterface)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::WaypointPolyBase)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::WaypointPoly)

BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::detail_waypoint::WaypointInterface)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::WaypointPolyBase)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::WaypointPoly)
