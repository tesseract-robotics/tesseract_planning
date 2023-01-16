#include <tesseract_motion_planners/core/profiles.h>

namespace tesseract_planning
{
template <class Archive>
void WaypointProfile::serialize(Archive&, const unsigned int)
{
}

template <class Archive>
void CompositeProfile::serialize(Archive&, const unsigned int)
{
}

template <class Archive>
void PlannerProfile::serialize(Archive&, const unsigned int)
{
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
BOOST_SERIALIZATION_ASSUME_ABSTRACT(tesseract_planning::WaypointProfile);
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::WaypointProfile);

BOOST_SERIALIZATION_ASSUME_ABSTRACT(tesseract_planning::CompositeProfile);
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::CompositeProfile);

BOOST_SERIALIZATION_ASSUME_ABSTRACT(tesseract_planning::PlannerProfile);
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::PlannerProfile);
