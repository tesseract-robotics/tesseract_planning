#ifndef TESSERACT_MOTION_PLANNERS_OMPL_PROFILE_OMPL_PLANNER_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_OMPL_PROFILE_OMPL_PLANNER_PROFILE_H

#include <tesseract_command_language/profile_dictionary.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>

namespace tesseract_planning
{
/**
 * @brief Planner profile that produces the high level parameters for the OMPL planner
 */
class OMPLPlannerProfile : public PlannerProfile
{
public:
  OMPLPlannerParameters params;
  inline std::any create() const override
  {
    if (params.planners.empty())
      throw std::runtime_error("No OMPL planner factories defined");
    return params;
  };

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int)
  {
  }
};

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_OMPL_PROFILE_OMPL_PLANNER_PROFILE_H
