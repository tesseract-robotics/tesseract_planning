#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/algorithm/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt/trajopt_utils.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_constraint_plan_profile.h>

#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/instruction_type.h>

namespace tesseract_planning
{
void TrajOptConstraintPlanProfile::apply(trajopt::ProblemConstructionInfo& pci,
                                         const CartesianWaypoint& cartesian_waypoint,
                                         const Instruction& parent_instruction,
                                         const ManipulatorInfo& manip_info,
                                         const std::vector<std::string>& active_links,
                                         int index) const
{
  assert(isPlanInstruction(parent_instruction));
  const auto& base_instruction = parent_instruction.as<PlanInstruction>();
  assert(!(manip_info.empty() && base_instruction.getManipulatorInfo().empty()));
  ManipulatorInfo mi = manip_info.getCombined(base_instruction.getManipulatorInfo());

  // TODO rename these checks later
  if (mi.manipulator.empty())
    throw std::runtime_error("TrajOptPlanProfile, manipulator is empty!");

  if (mi.tcp_frame.empty())
    throw std::runtime_error("TrajOptPlanProfile, tcp_frame is empty!");

  if (mi.working_frame.empty())
    throw std::runtime_error("TrajOptPlanProfile, working_frame is empty!");

  Eigen::Isometry3d tcp_offset = pci.env->findTCPOffset(mi);

  trajopt::TermInfo::Ptr ti{ nullptr };

  bool is_active_tcp_frame = (std::find(active_links.begin(), active_links.end(), mi.tcp_frame) != active_links.end());
  bool is_static_working_frame =
      (std::find(active_links.begin(), active_links.end(), mi.working_frame) == active_links.end());

  if (cartesian_waypoint.isToleranced())
    CONSOLE_BRIDGE_logWarn("Tolerance cartesian waypoints are not supported in this version of TrajOpt.");

  if ((is_static_working_frame && is_active_tcp_frame) || (!is_active_tcp_frame && !is_static_working_frame))
  {
    ti = createCartesianWaypointTermInfo(
        index, mi.working_frame, cartesian_waypoint, mi.tcp_frame, tcp_offset, cartesian_coeff, term_type);
  }
  else if (!is_static_working_frame && is_active_tcp_frame)
  {
    ti = createDynamicCartesianWaypointTermInfo(
        index, mi.working_frame, cartesian_waypoint, mi.tcp_frame, tcp_offset, cartesian_coeff, term_type);
  }
  else
  {
    throw std::runtime_error("TrajOpt, both tcp_frame and working_frame are both static!");
  }

  if (term_type == trajopt::TermType::TT_CNT)
    pci.cnt_infos.push_back(ti);
  else
    pci.cost_infos.push_back(ti);

  // Add constraints from error functions if available.
  addConstraintErrorFunctions(pci, index);

  // Add new constraints
  Eigen::VectorXd new_coeff{ Eigen::VectorXd::Constant(6, 1, 5) };

  new_coeff(0) = 0;
  new_coeff(1) = 0;
  new_coeff(2) = 0;

  ti = createDynamicCartesianWaypointTermInfo(
      index, "world", cartesian_waypoint, mi.tcp_frame, tcp_offset, new_coeff, term_type);
  pci.cnt_infos.push_back(ti);
}
}  // namespace tesseract_planning
