#include <tesseract_motion_planners/ompl/profile/ompl_waypoint_profile.h>
#include <tesseract_kinematics/core/utils.h>

namespace tesseract_planning
{
void checkCollision(const Eigen::VectorXd& state,
                    const tesseract_environment::Environment& env,
                    const tesseract_kinematics::JointGroup::ConstPtr& manip)
{
  tesseract_collision::DiscreteContactManager::Ptr contact_checker = env.getDiscreteContactManager();
  tesseract_common::TransformMap link_transforms = manip->calcFwdKin(state);

  for (const auto& link_name : contact_checker->getActiveCollisionObjects())
    contact_checker->setCollisionObjectsTransform(link_name, link_transforms[link_name]);

  tesseract_collision::ContactResultMap contact_map;
  contact_checker->contactTest(contact_map, tesseract_collision::ContactTestType::FIRST);

  if (!contact_map.empty())
  {
    std::stringstream ss;
    ss << "State is in collision";
    for (const auto& contact_vec : contact_map)
      for (const auto& contact : contact_vec.second)
        ss << "\n\tLinks: " << contact.link_names[0] << ", " << contact.link_names[1]
           << " Distance: " << contact.distance;
    throw std::runtime_error(ss.str());
  }
}

Eigen::VectorXd updateLimits(const Eigen::Ref<const Eigen::VectorXd>& joint_waypoint,
                             const tesseract_common::KinematicLimits& limits)
{
  if (!tesseract_common::satisfiesPositionLimits(joint_waypoint, limits.joint_limits))
    throw std::runtime_error("State violates joint limits");

  Eigen::VectorXd tmp(joint_waypoint);
  tesseract_common::enforcePositionLimits(tmp, limits.joint_limits);
  return tmp;
}

tesseract_kinematics::IKSolutions getValidIKSolutions(const Eigen::Isometry3d& cartesian_waypoint,
                                                      const ManipulatorInfo& mi,
                                                      const tesseract_environment::Environment& env)
{
  if (mi.manipulator.empty())
    throw std::runtime_error("OMPL: manipulator is empty!");

  if (mi.tcp_frame.empty())
    throw std::runtime_error("OMPL: tcp_frame is empty!");

  if (mi.working_frame.empty())
    throw std::runtime_error("OMPL: working_frame is empty!");

  // Get the kinematics group for solving IK
  tesseract_kinematics::KinematicGroup::ConstPtr manip =
      env.getKinematicGroup(mi.manipulator, mi.manipulator_ik_solver);

  Eigen::Isometry3d tcp_offset = env.findTCPOffset(mi);
  Eigen::Isometry3d tcp_frame_cwp = cartesian_waypoint * tcp_offset.inverse();
  tesseract_kinematics::KinGroupIKInput ik_input(tcp_frame_cwp, mi.working_frame, mi.tcp_frame);
  const tesseract_kinematics::IKSolutions joint_solutions =
      manip->calcInvKin({ ik_input }, Eigen::VectorXd::Zero(manip->numJoints()));

  // Assemble all the valid solutionss
  tesseract_common::KinematicLimits limits = manip->getLimits();
  tesseract_kinematics::IKSolutions valid_solutions;
  valid_solutions.reserve(joint_solutions.size());
  for (const Eigen::VectorXd& js : joint_solutions)
  {
    try
    {
      // Update the joint solution based on the kinematic limits
      Eigen::VectorXd solution = updateLimits(js, limits);

      // Ensure the solution is collision-free
      checkCollision(solution, env, manip);

      // Add the solution to the container
      valid_solutions.push_back(solution);

      // Add the redundant solutions
      auto redundant_solutions = tesseract_kinematics::getRedundantSolutions<double>(
          solution, limits.joint_limits, manip->getRedundancyCapableJointIndices());
      valid_solutions.insert(valid_solutions.end(), redundant_solutions.begin(), redundant_solutions.end());
    }
    catch (const std::exception& ex)
    {
      CONSOLE_BRIDGE_logDebug(ex.what());
      continue;
    }
  }

  if (valid_solutions.empty())
    throw std::runtime_error("All states are either in collision or outside limits");

  return valid_solutions;
}

std::any OMPLWaypointProfile::create(const Instruction& instruction,
                                     const tesseract_environment::Environment& env) const
{
  const auto& plan_instruction = instruction.as<PlanInstruction>();
  const tesseract_common::ManipulatorInfo& mi = plan_instruction.getManipulatorInfo();
  const Waypoint& waypoint = plan_instruction.getWaypoint();

  if (isCartesianWaypoint(waypoint))
  {
    const auto& cw = waypoint.as<CartesianWaypoint>();
    return getValidIKSolutions(cw, mi, env);
  }

  if (isJointWaypoint(waypoint))
  {
    const auto& jw = waypoint.as<JointWaypoint>();
    const Eigen::VectorXd updated_state = updateLimits(jw, env.getJointGroup(mi.manipulator)->getLimits());
    checkCollision(updated_state, env, env.getJointGroup(mi.manipulator));
    return std::vector<Eigen::VectorXd>{ updated_state };
  }

  if (isStateWaypoint(waypoint))
  {
    const auto& sw = waypoint.as<StateWaypoint>();
    Eigen::Map<const Eigen::VectorXd> state(sw.position.data(), sw.position.size());
    const Eigen::VectorXd updated_state = updateLimits(state, env.getJointGroup(mi.manipulator)->getLimits());
    checkCollision(updated_state, env, env.getJointGroup(mi.manipulator));
    return std::vector<Eigen::VectorXd>{ updated_state };
  }

  throw std::runtime_error("Unsupported waypoint type");
}

}  // namespace tesseract_planning
