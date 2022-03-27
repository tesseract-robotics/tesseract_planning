#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/nvp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/serialization.h>
#include <tesseract_command_language/core/waypoint.h>

template <class Archive>
void tesseract_planning::detail_waypoint::WaypointInnerBase::serialize(Archive& /*ar*/, const unsigned int /*version*/)  // NOLINT
{
}

tesseract_planning::Waypoint::Waypoint()  // NOLINT
  : waypoint_(nullptr)
{
}

tesseract_planning::Waypoint::Waypoint(const Waypoint& other) { waypoint_ = other.waypoint_->clone(); }

tesseract_planning::Waypoint::Waypoint(Waypoint&& other) noexcept { waypoint_.swap(other.waypoint_); }

tesseract_planning::Waypoint& tesseract_planning::Waypoint::operator=(Waypoint&& other) noexcept
{
  waypoint_.swap(other.waypoint_);
  return (*this);
}

tesseract_planning::Waypoint& tesseract_planning::Waypoint::operator=(const Waypoint& other)
{
  (*this) = Waypoint(other);
  return (*this);
}

std::type_index tesseract_planning::Waypoint::getType() const { return waypoint_->getType(); }

void tesseract_planning::Waypoint::print(const std::string& prefix) const { waypoint_->print(prefix); }

bool tesseract_planning::Waypoint::operator==(const Waypoint& rhs) const
{
  return waypoint_->operator==(*rhs.waypoint_);
}

bool tesseract_planning::Waypoint::operator!=(const Waypoint& rhs) const { return !operator==(rhs); }

template <class Archive>
void tesseract_planning::Waypoint::serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
{
  ar& boost::serialization::make_nvp("waypoint", waypoint_);
}

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::detail_waypoint::WaypointInnerBase)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::Waypoint)
