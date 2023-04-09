/**
 * @file utils_test.cpp
 * @brief
 *
 * @author Matthew Powelson
 * @date June 15, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>
#include <tesseract_environment/environment.h>
#include <tesseract_environment/commands/add_link_command.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/planner_utils.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_geometry/impl/sphere.h>
#include <tesseract_support/tesseract_support_resource_locator.h>

using namespace tesseract_planning;
using namespace tesseract_environment;

class TesseractPlanningUtilsUnit : public ::testing::Test
{
protected:
  Environment::Ptr env_;

  void SetUp() override
  {
    auto locator = std::make_shared<tesseract_common::TesseractSupportResourceLocator>();
    Environment::Ptr env = std::make_shared<Environment>();
    tesseract_common::fs::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf");
    tesseract_common::fs::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf");
    EXPECT_TRUE(env->init(urdf_path, srdf_path, locator));
    env_ = env;
  }
};

TEST_F(TesseractPlanningUtilsUnit, GenerateSeed)  // NOLINT
{
  EXPECT_TRUE(true);
}

TEST_F(TesseractPlanningUtilsUnit, GetProfileStringTest)  // NOLINT
{
  std::string input_profile;
  std::string planner_name = "Planner_1";
  std::string default_planner = "TEST_DEFAULT";

  std::unordered_map<std::string, std::string> remap;
  remap["profile_1"] = "profile_1_remapped";
  PlannerProfileRemapping remapping;
  remapping["Planner_2"] = remap;

  // Empty input profile
  std::string output_profile = getProfileString(planner_name, input_profile, remapping);
  EXPECT_EQ(output_profile, "DEFAULT");
  output_profile = getProfileString(planner_name, input_profile, remapping, default_planner);
  EXPECT_EQ(output_profile, default_planner);

  // Planner name doesn't match
  input_profile = "profile_1";
  output_profile = getProfileString(planner_name, input_profile, remapping, default_planner);
  EXPECT_EQ(input_profile, output_profile);

  // Profile name doesn't match
  input_profile = "doesnt_match";
  output_profile = getProfileString("Planner_2", input_profile, remapping, default_planner);
  EXPECT_EQ(input_profile, output_profile);

  // Successful remap
  input_profile = "profile_1";
  output_profile = getProfileString("Planner_2", input_profile, remapping, default_planner);
  EXPECT_EQ(output_profile, "profile_1_remapped");
}

void checkProcessInterpolatedResults(const std::vector<tesseract_collision::ContactResultMap>& contacts)
{
  for (const auto& c : contacts)
  {
    for (const auto& pair : c)
    {
      for (const auto& r : pair.second)
      {
        for (std::size_t j = 0; j < 2; ++j)
        {
          if (!(r.cc_time[j] < 0))
          {
            if (tesseract_common::almostEqualRelativeAndAbs(r.cc_time[j], 0.0))
              EXPECT_EQ(r.cc_type[j], tesseract_collision::ContinuousCollisionType::CCType_Time0);
            else if (tesseract_common::almostEqualRelativeAndAbs(r.cc_time[j], 1.0))
              EXPECT_EQ(r.cc_type[j], tesseract_collision::ContinuousCollisionType::CCType_Time1);
            else
              EXPECT_EQ(r.cc_type[j], tesseract_collision::ContinuousCollisionType::CCType_Between);
          }
          else
          {
            EXPECT_EQ(r.cc_type[j], tesseract_collision::ContinuousCollisionType::CCType_None);
          }
        }

        EXPECT_TRUE((r.cc_type[0] != tesseract_collision::ContinuousCollisionType::CCType_None ||
                     r.cc_type[1] != tesseract_collision::ContinuousCollisionType::CCType_None));
      }
    }
  }
}

/**
 * @brief Verify that no contact results is at Time0
 * @param contacts The contacts to check
 */
void checkProcessInterpolatedResultsNoTime0(const tesseract_collision::ContactResultMap& contacts)
{
  for (const auto& pair : contacts)
  {
    for (const auto& r : pair.second)
    {
      EXPECT_NE(r.cc_type[0], tesseract_collision::ContinuousCollisionType::CCType_Time0);
      EXPECT_NE(r.cc_type[1], tesseract_collision::ContinuousCollisionType::CCType_Time0);
    }
  }
}

/**
 * @brief Verify that no contact results is at Time1
 * @param contacts The contacts to check
 */
void checkProcessInterpolatedResultsNoTime1(const tesseract_collision::ContactResultMap& contacts)
{
  for (const auto& pair : contacts)
  {
    for (const auto& r : pair.second)
    {
      EXPECT_NE(r.cc_type[0], tesseract_collision::ContinuousCollisionType::CCType_Time1);
      EXPECT_NE(r.cc_type[1], tesseract_collision::ContinuousCollisionType::CCType_Time1);
    }
  }
}

/**
 * @brief Verify that the results contain Time0
 * @param contacts The contacts to check
 */
bool hasProcessInterpolatedResultsTime0(const tesseract_collision::ContactResultMap& contacts)
{
  for (const auto& pair : contacts)
  {
    for (const auto& r : pair.second)
    {
      if (r.cc_type[0] == tesseract_collision::ContinuousCollisionType::CCType_Time0)
        return true;

      if (r.cc_type[1] == tesseract_collision::ContinuousCollisionType::CCType_Time0)
        return true;
    }
  }
  return false;
}

/**
 * @brief Verify that the results contain Time1
 * @param contacts The contacts to check
 */
bool hasProcessInterpolatedResultsTime1(const tesseract_collision::ContactResultMap& contacts)
{
  for (const auto& pair : contacts)
  {
    for (const auto& r : pair.second)
    {
      if (r.cc_type[0] == tesseract_collision::ContinuousCollisionType::CCType_Time1)
        return true;

      if (r.cc_type[1] == tesseract_collision::ContinuousCollisionType::CCType_Time1)
        return true;
    }
  }
  return false;
}

/** @brief Get the total number of contacts */
long getContactCount(const std::vector<tesseract_collision::ContactResultMap>& contacts)
{
  long total{ 0 };
  for (const auto& c : contacts)
    total += c.count();

  return total;
}

TEST_F(TesseractPlanningUtilsUnit, checkProgramUnit)  // NOLINT
{
  // Add sphere to environment
  tesseract_scene_graph::Link link_sphere("sphere_attached");

  tesseract_scene_graph::Visual::Ptr visual = std::make_shared<tesseract_scene_graph::Visual>();
  visual->origin = Eigen::Isometry3d::Identity();
  visual->origin.translation() = Eigen::Vector3d(0.5, 0, 0.55);
  visual->geometry = std::make_shared<tesseract_geometry::Sphere>(0.15);
  link_sphere.visual.push_back(visual);

  tesseract_scene_graph::Collision::Ptr collision = std::make_shared<tesseract_scene_graph::Collision>();
  collision->origin = visual->origin;
  collision->geometry = visual->geometry;
  link_sphere.collision.push_back(collision);

  tesseract_scene_graph::Joint joint_sphere("joint_sphere_attached");
  joint_sphere.parent_link_name = "base_link";
  joint_sphere.child_link_name = link_sphere.getName();
  joint_sphere.type = tesseract_scene_graph::JointType::FIXED;

  auto cmd = std::make_shared<tesseract_environment::AddLinkCommand>(link_sphere, joint_sphere);

  EXPECT_TRUE(env_->applyCommand(cmd));

  // Set the robot initial state
  std::vector<std::string> joint_names;
  joint_names.emplace_back("joint_a1");
  joint_names.emplace_back("joint_a2");
  joint_names.emplace_back("joint_a3");
  joint_names.emplace_back("joint_a4");
  joint_names.emplace_back("joint_a5");
  joint_names.emplace_back("joint_a6");
  joint_names.emplace_back("joint_a7");

  Eigen::VectorXd joint_start_pos(7);
  joint_start_pos(0) = -0.4;
  joint_start_pos(1) = 0.2762;
  joint_start_pos(2) = 0.0;
  joint_start_pos(3) = -1.3348;
  joint_start_pos(4) = 0.0;
  joint_start_pos(5) = 1.4959;
  joint_start_pos(6) = 0.0;

  Eigen::VectorXd joint_end_pos(7);
  joint_end_pos(0) = 0.4;
  joint_end_pos(1) = 0.2762;
  joint_end_pos(2) = 0.0;
  joint_end_pos(3) = -1.3348;
  joint_end_pos(4) = 0.0;
  joint_end_pos(5) = 1.4959;
  joint_end_pos(6) = 0.0;

  Eigen::VectorXd joint_pos_collision(7);
  joint_pos_collision(0) = 0.0;
  joint_pos_collision(1) = 0.2762;
  joint_pos_collision(2) = 0.0;
  joint_pos_collision(3) = -1.3348;
  joint_pos_collision(4) = 0.0;
  joint_pos_collision(5) = 1.4959;
  joint_pos_collision(6) = 0.0;

  using tesseract_planning::CompositeInstruction;
  using tesseract_planning::contactCheckProgram;
  using tesseract_planning::MoveInstruction;
  using tesseract_planning::MoveInstructionType;
  using tesseract_planning::StateWaypoint;
  using tesseract_planning::StateWaypointPoly;

  // Only intermediat states are in collision
  tesseract_common::TrajArray traj(5, joint_start_pos.size());
  for (int i = 0; i < joint_start_pos.size(); ++i)
    traj.col(i) = Eigen::VectorXd::LinSpaced(5, joint_start_pos(i), joint_end_pos(i));

  CompositeInstruction traj_ci;
  for (long r = 0; r < traj.rows(); ++r)
  {
    StateWaypointPoly swp{ StateWaypoint(joint_names, traj.row(r)) };
    traj_ci.appendMoveInstruction(MoveInstruction(swp, MoveInstructionType::FREESPACE));
  }

  // Only start state is not in collision
  tesseract_common::TrajArray traj2(3, joint_start_pos.size());
  for (int i = 0; i < joint_start_pos.size(); ++i)
    traj2.col(i) = Eigen::VectorXd::LinSpaced(3, joint_start_pos(i), joint_pos_collision(i));

  CompositeInstruction traj2_ci;
  for (long r = 0; r < traj2.rows(); ++r)
  {
    StateWaypointPoly swp{ StateWaypoint(joint_names, traj2.row(r)) };
    traj2_ci.appendMoveInstruction(MoveInstruction(swp, MoveInstructionType::FREESPACE));
  }

  // Only start state is not in collision
  tesseract_common::TrajArray traj3(3, joint_start_pos.size());
  for (int i = 0; i < joint_start_pos.size(); ++i)
    traj3.col(i) = Eigen::VectorXd::LinSpaced(3, joint_pos_collision(i), joint_end_pos(i));

  CompositeInstruction traj3_ci;
  for (long r = 0; r < traj3.rows(); ++r)
  {
    StateWaypointPoly swp{ StateWaypoint(joint_names, traj3.row(r)) };
    traj3_ci.appendMoveInstruction(MoveInstruction(swp, MoveInstructionType::FREESPACE));
  }

  // Only two states
  tesseract_common::TrajArray traj4(2, joint_start_pos.size());
  traj4.row(0) = joint_pos_collision;
  traj4.row(1) = joint_end_pos;

  CompositeInstruction traj4_ci;
  for (long r = 0; r < traj4.rows(); ++r)
  {
    StateWaypointPoly swp{ StateWaypoint(joint_names, traj4.row(r)) };
    traj4_ci.appendMoveInstruction(MoveInstruction(swp, MoveInstructionType::FREESPACE));
  }

  auto discrete_manager = env_->getDiscreteContactManager();
  auto continuous_manager = env_->getContinuousContactManager();
  auto state_solver = env_->getStateSolver();

  {  // CollisionEvaluatorType::DISCRETE && CollisionCheckProgramType::ALL
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::DISCRETE;
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::ALL;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj_ci.size()));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(contacts.at(2).size(), 2);
    EXPECT_EQ(contacts.at(3).size(), 2);
    EXPECT_EQ(contacts.at(4).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(6));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj2_ci.size()));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(contacts.at(2).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(4));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(2)));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj2_ci.size()));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(contacts.at(2).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(4));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(0)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    checkProcessInterpolatedResults(contacts);
  }

  {  // CollisionEvaluatorType::DISCRETE && CollisionCheckProgramType::ALL_EXCEPT_END
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::DISCRETE;
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_END;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(contacts.at(2).size(), 2);
    EXPECT_EQ(contacts.at(3).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(6));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(2)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(3)));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj2_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(2));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj2_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(4));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(0)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    checkProcessInterpolatedResults(contacts);
  }

  {  // CollisionEvaluatorType::DISCRETE && CollisionCheckProgramType::ALL_EXCEPT_START
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::DISCRETE;
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_START;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj_ci.size()));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(contacts.at(2).size(), 2);
    EXPECT_EQ(contacts.at(3).size(), 2);
    EXPECT_EQ(contacts.at(4).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(6));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(2)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(3)));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj2_ci.size()));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(contacts.at(2).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(4));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(2)));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj2_ci.size()));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(contacts.at(2).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(2));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    checkProcessInterpolatedResults(contacts);
  }

  {  // CollisionEvaluatorType::DISCRETE && CollisionCheckProgramType::INTERMEDIATE_ONLY
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::DISCRETE;
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(contacts.at(2).size(), 2);
    EXPECT_EQ(contacts.at(3).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(6));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(2)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(3)));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj2_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(2));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj2_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(2));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    checkProcessInterpolatedResults(contacts);
  }

  {  // CollisionEvaluatorType::DISCRETE && CollisionCheckProgramType::END_ONLY
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::DISCRETE;
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::END_ONLY;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_FALSE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), 1);
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(0));

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), 1);
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(2));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(0)));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_FALSE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), 1);
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(0));
  }

  {  // CollisionEvaluatorType::DISCRETE && CollisionCheckProgramType::START_ONLY
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::DISCRETE;
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::START_ONLY;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_FALSE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), 1);
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(0));

    contacts.clear();
    EXPECT_FALSE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), 1);
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(0));

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), 1);
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(2));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(0)));
    checkProcessInterpolatedResults(contacts);
  }

  {
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::DISCRETE;
    config.contact_request.type = tesseract_collision::ContactTestType::FIRST;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), 2);
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(contacts.at(1).size(), 1);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(1));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), 2);
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(contacts.at(1).size(), 1);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(1));
    checkProcessInterpolatedResults(contacts);
  }

  {
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::DISCRETE;
    std::vector<tesseract_collision::ContactResultMap> contacts;

    CompositeInstruction ci;
    StateWaypointPoly swp{ StateWaypoint(joint_names, joint_start_pos) };
    ci.appendMoveInstruction(MoveInstruction(swp, MoveInstructionType::FREESPACE));

    EXPECT_FALSE(contactCheckProgram(contacts, *discrete_manager, *state_solver, ci, config));
    EXPECT_EQ(contacts.size(), 1);
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(0));
    checkProcessInterpolatedResults(contacts);
  }

  {  // CollisionEvaluatorType::LVS_DISCRETE && CollisionCheckProgramType::ALL
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
    config.longest_valid_segment_length = std::numeric_limits<double>::max();
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::ALL;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(contacts.at(2).size(), 2);
    EXPECT_EQ(contacts.at(3).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(6));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(2)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(3)));
    checkProcessInterpolatedResultsNoTime1(contacts.at(3));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj2_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(4));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime1(contacts.at(1)));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj3_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(4));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(0)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    checkProcessInterpolatedResultsNoTime1(contacts.at(0));
    checkProcessInterpolatedResultsNoTime1(contacts.at(1));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj4_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj4_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(2));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(0)));
    checkProcessInterpolatedResultsNoTime1(contacts.at(0));
    checkProcessInterpolatedResults(contacts);
  }

  {  // CollisionEvaluatorType::LVS_DISCRETE && CollisionCheckProgramType::ALL_EXCEPT_END
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
    config.longest_valid_segment_length = std::numeric_limits<double>::max();
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_END;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(contacts.at(2).size(), 2);
    EXPECT_EQ(contacts.at(3).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(6));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(2)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(3)));
    checkProcessInterpolatedResultsNoTime1(contacts.at(3));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj2_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(2));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    checkProcessInterpolatedResultsNoTime1(contacts.at(1));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj3_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(4));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(0)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    checkProcessInterpolatedResultsNoTime1(contacts.at(0));
    checkProcessInterpolatedResultsNoTime1(contacts.at(1));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj4_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj4_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(2));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(0)));
    checkProcessInterpolatedResultsNoTime1(contacts.at(0));
    checkProcessInterpolatedResults(contacts);
  }

  {  // CollisionEvaluatorType::LVS_DISCRETE && CollisionCheckProgramType::ALL_EXCEPT_START
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
    config.longest_valid_segment_length = std::numeric_limits<double>::max();
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_START;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(contacts.at(2).size(), 2);
    EXPECT_EQ(contacts.at(3).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(6));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(2)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(3)));
    checkProcessInterpolatedResultsNoTime1(contacts.at(3));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj2_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(4));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime1(contacts.at(1)));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj3_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(2));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    checkProcessInterpolatedResultsNoTime1(contacts.at(1));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_FALSE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj4_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj4_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(0));
  }

  {  // CollisionEvaluatorType::LVS_DISCRETE && CollisionCheckProgramType::INTERMEDIATE_ONLY
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
    config.longest_valid_segment_length = std::numeric_limits<double>::max();
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(contacts.at(2).size(), 2);
    EXPECT_EQ(contacts.at(3).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(6));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(2)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(3)));
    checkProcessInterpolatedResultsNoTime1(contacts.at(3));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj2_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(2));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    checkProcessInterpolatedResultsNoTime1(contacts.at(1));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj3_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(2));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    checkProcessInterpolatedResultsNoTime1(contacts.at(1));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_FALSE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj4_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj4_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(0));
  }

  {  // CollisionEvaluatorType::LVS_DISCRETE && CollisionCheckProgramType::END_ONLY
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
    config.longest_valid_segment_length = std::numeric_limits<double>::max();
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::END_ONLY;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_FALSE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(1));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(0));

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(1));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(2));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(0)));
    checkProcessInterpolatedResultsNoTime1(contacts.at(0));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_FALSE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(1));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(0));
  }

  {  // CollisionEvaluatorType::LVS_DISCRETE && CollisionCheckProgramType::START_ONLY
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
    config.longest_valid_segment_length = std::numeric_limits<double>::max();
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::START_ONLY;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_FALSE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(1));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(0));

    contacts.clear();
    EXPECT_FALSE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(1));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(0));

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(1));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(2));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(0)));
    checkProcessInterpolatedResultsNoTime1(contacts.at(0));
    checkProcessInterpolatedResults(contacts);
  }

  {
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
    config.contact_request.type = tesseract_collision::ContactTestType::FIRST;
    config.longest_valid_segment_length = std::numeric_limits<double>::max();
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), 2);
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(contacts.at(1).size(), 1);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(1));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), 2);
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(contacts.at(1).size(), 1);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(1));
    checkProcessInterpolatedResults(contacts);
  }

  {  // CollisionEvaluatorType::LVS_DISCRETE && CollisionCheckProgramType::ALL
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::ALL;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(contacts.at(2).size(), 3);
    EXPECT_EQ(contacts.at(3).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(285));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(2)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(3)));
    checkProcessInterpolatedResultsNoTime1(contacts.at(3));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj2_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(125));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime1(contacts.at(1)));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj3_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 3);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(160));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(0)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    checkProcessInterpolatedResultsNoTime1(contacts.at(1));
    checkProcessInterpolatedResults(contacts);
  }

  {  // CollisionEvaluatorType::LVS_DISCRETE && CollisionCheckProgramType::ALL_EXCEPT_END
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_END;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(contacts.at(2).size(), 3);
    EXPECT_EQ(contacts.at(3).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(285));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(2)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(3)));
    checkProcessInterpolatedResultsNoTime1(contacts.at(3));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj2_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(123));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    checkProcessInterpolatedResultsNoTime1(contacts.at(1));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj3_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 3);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(160));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(0)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    checkProcessInterpolatedResultsNoTime1(contacts.at(1));
    checkProcessInterpolatedResults(contacts);
  }

  {  // CollisionEvaluatorType::LVS_DISCRETE && CollisionCheckProgramType::ALL_EXCEPT_START
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_START;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(contacts.at(2).size(), 3);
    EXPECT_EQ(contacts.at(3).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(285));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(2)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(3)));
    checkProcessInterpolatedResultsNoTime1(contacts.at(3));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj2_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(125));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime1(contacts.at(1)));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj3_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 3);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(158));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    checkProcessInterpolatedResultsNoTime1(contacts.at(1));
    checkProcessInterpolatedResults(contacts);
  }

  {  // CollisionEvaluatorType::LVS_DISCRETE && CollisionCheckProgramType::INTERMEDIATE_ONLY
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(contacts.at(2).size(), 3);
    EXPECT_EQ(contacts.at(3).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(285));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(2)));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(3)));
    checkProcessInterpolatedResultsNoTime1(contacts.at(3));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj2_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(123));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    checkProcessInterpolatedResultsNoTime1(contacts.at(1));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj3_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 3);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(158));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(1)));
    checkProcessInterpolatedResultsNoTime1(contacts.at(1));
    checkProcessInterpolatedResults(contacts);
  }

  {  // CollisionEvaluatorType::LVS_DISCRETE && CollisionCheckProgramType::END_ONLY
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::END_ONLY;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_FALSE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(1));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(0));

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(1));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(2));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(0)));
    checkProcessInterpolatedResultsNoTime1(contacts.at(0));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_FALSE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(1));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(0));
  }

  {  // CollisionEvaluatorType::LVS_DISCRETE && CollisionCheckProgramType::START_ONLY
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::START_ONLY;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_FALSE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(1));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(0));

    contacts.clear();
    EXPECT_FALSE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(1));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(0));

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(1));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(2));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime0(contacts.at(0)));
    checkProcessInterpolatedResults(contacts);
  }

  {  // CollisionEvaluatorType::CONTINUOUS && CollisionCheckProgramType::ALL
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::CONTINUOUS;
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::ALL;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(contacts.at(2).size(), 3);
    EXPECT_EQ(contacts.at(3).size(), 2);
    /**< @todo we are getting 9 instead of 6 because duplicate contacts at states */
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(9));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    checkProcessInterpolatedResultsNoTime1(contacts.at(3));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj2_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(4));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    checkProcessInterpolatedResultsNoTime1(contacts.at(1));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj3_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 3);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(5));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    checkProcessInterpolatedResultsNoTime1(contacts.at(1));
    checkProcessInterpolatedResults(contacts);
  }

  {  // CollisionEvaluatorType::CONTINUOUS && CollisionCheckProgramType::ALL_EXCEPT_END
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::CONTINUOUS;
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_END;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj_ci.size() - 2));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(contacts.at(2).size(), 3);
    /**< @todo we are getting 9 instead of 6 because duplicate contacts at states */
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(7));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    checkProcessInterpolatedResultsNoTime1(contacts.at(2));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj2_ci.size() - 2));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(2));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime1(contacts.at(0)));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj3_ci.size() - 2));
    EXPECT_EQ(contacts.at(0).size(), 3);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(3));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    checkProcessInterpolatedResultsNoTime1(contacts.at(0));
    checkProcessInterpolatedResults(contacts);
  }

  {  // CollisionEvaluatorType::CONTINUOUS && CollisionCheckProgramType::ALL_EXCEPT_START
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::CONTINUOUS;
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_START;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(contacts.at(2).size(), 3);
    EXPECT_EQ(contacts.at(3).size(), 2);
    /**< @todo we are getting 7 instead of 4 because duplicate contacts at states */
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(7));
    checkProcessInterpolatedResultsNoTime1(contacts.at(3));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj2_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(2));
    checkProcessInterpolatedResultsNoTime0(contacts.at(1));
    checkProcessInterpolatedResultsNoTime1(contacts.at(1));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj3_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(2));
    checkProcessInterpolatedResultsNoTime1(contacts.at(1));
    checkProcessInterpolatedResults(contacts);
  }

  {  // CollisionEvaluatorType::CONTINUOUS && CollisionCheckProgramType::INTERMEDIATE_ONLY
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::CONTINUOUS;
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj_ci.size() - 2));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(contacts.at(2).size(), 3);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(5));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    checkProcessInterpolatedResultsNoTime1(contacts.at(2));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_FALSE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj2_ci.size() - 2));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(0));

    contacts.clear();
    EXPECT_FALSE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj3_ci.size() - 2));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(0));
  }

  {  // CollisionEvaluatorType::CONTINUOUS && CollisionCheckProgramType::END_ONLY
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::CONTINUOUS;
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::END_ONLY;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_FALSE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), 1);
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(0));

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), 1);
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(2));

    contacts.clear();
    EXPECT_FALSE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), 1);
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(0));
  }

  {  // CollisionEvaluatorType::CONTINUOUS && CollisionCheckProgramType::START_ONLY
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::CONTINUOUS;
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::START_ONLY;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_FALSE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), 1);
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(0));

    contacts.clear();
    EXPECT_FALSE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), 1);
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(0));

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), 1);
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(2));
  }

  {
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::CONTINUOUS;
    config.contact_request.type = tesseract_collision::ContactTestType::FIRST;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), 1);
    EXPECT_EQ(contacts.at(0).size(), 1);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(1));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), 1);
    EXPECT_EQ(contacts.at(0).size(), 1);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(1));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    checkProcessInterpolatedResults(contacts);
  }

  {  // CollisionEvaluatorType::LVS_CONTINUOUS && CollisionCheckProgramType::ALL
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS;
    config.longest_valid_segment_length = std::numeric_limits<double>::max();
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::ALL;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(contacts.at(2).size(), 3);
    EXPECT_EQ(contacts.at(3).size(), 2);
    /**< @todo we are getting 9 instead of 6 because duplicate contacts at states */
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(9));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    checkProcessInterpolatedResultsNoTime1(contacts.at(3));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj2_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(4));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    checkProcessInterpolatedResultsNoTime1(contacts.at(1));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj3_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 3);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(5));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    checkProcessInterpolatedResultsNoTime1(contacts.at(1));
    checkProcessInterpolatedResults(contacts);
  }

  {  // CollisionEvaluatorType::LVS_CONTINUOUS && CollisionCheckProgramType::ALL_EXCEPT_END
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS;
    config.longest_valid_segment_length = std::numeric_limits<double>::max();
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_END;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj_ci.size() - 2));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(contacts.at(2).size(), 3);
    /**< @todo we are getting 9 instead of 6 because duplicate contacts at states */
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(7));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    checkProcessInterpolatedResultsNoTime1(contacts.at(2));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj2_ci.size() - 2));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(2));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    EXPECT_TRUE(hasProcessInterpolatedResultsTime1(contacts.at(0)));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj3_ci.size() - 2));
    EXPECT_EQ(contacts.at(0).size(), 3);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(3));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    checkProcessInterpolatedResultsNoTime1(contacts.at(0));
    checkProcessInterpolatedResults(contacts);
  }

  {  // CollisionEvaluatorType::LVS_CONTINUOUS && CollisionCheckProgramType::ALL_EXCEPT_START
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS;
    config.longest_valid_segment_length = std::numeric_limits<double>::max();
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_START;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(contacts.at(2).size(), 3);
    EXPECT_EQ(contacts.at(3).size(), 2);
    /**< @todo we are getting 7 instead of 4 because duplicate contacts at states */
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(7));
    checkProcessInterpolatedResultsNoTime1(contacts.at(3));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj2_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(2));
    checkProcessInterpolatedResultsNoTime0(contacts.at(1));
    checkProcessInterpolatedResultsNoTime1(contacts.at(1));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj3_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(2));
    checkProcessInterpolatedResultsNoTime1(contacts.at(1));
    checkProcessInterpolatedResults(contacts);
  }

  {  // CollisionEvaluatorType::LVS_CONTINUOUS && CollisionCheckProgramType::INTERMEDIATE_ONLY
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS;
    config.longest_valid_segment_length = std::numeric_limits<double>::max();
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj_ci.size() - 2));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(contacts.at(2).size(), 3);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(5));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    checkProcessInterpolatedResultsNoTime1(contacts.at(2));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_FALSE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj2_ci.size() - 2));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(0));

    contacts.clear();
    EXPECT_FALSE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj3_ci.size() - 2));
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(0));
  }

  {  // CollisionEvaluatorType::LVS_CONTINUOUS && CollisionCheckProgramType::END_ONLY
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS;
    config.longest_valid_segment_length = std::numeric_limits<double>::max();
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::END_ONLY;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_FALSE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), 1);
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(0));

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), 1);
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(2));

    contacts.clear();
    EXPECT_FALSE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), 1);
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(0));
  }

  {  // CollisionEvaluatorType::LVS_CONTINUOUS && CollisionCheckProgramType::START_ONLY
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS;
    config.longest_valid_segment_length = std::numeric_limits<double>::max();
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::START_ONLY;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_FALSE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), 1);
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(0));

    contacts.clear();
    EXPECT_FALSE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), 1);
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(0));

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), 1);
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(2));
  }

  {
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS;
    config.contact_request.type = tesseract_collision::ContactTestType::FIRST;
    config.longest_valid_segment_length = std::numeric_limits<double>::max();
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), 1);
    EXPECT_EQ(contacts.at(0).size(), 1);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(1));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), 1);
    EXPECT_EQ(contacts.at(0).size(), 1);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(1));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    checkProcessInterpolatedResults(contacts);
  }

  {  // CollisionEvaluatorType::LVS_CONTINUOUS && CollisionCheckProgramType::ALL
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS;
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::ALL;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(contacts.at(2).size(), 3);
    EXPECT_EQ(contacts.at(3).size(), 2);
    /**< @todo we are getting 288 instead of 285 because duplicate contacts at states */
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(288));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    checkProcessInterpolatedResultsNoTime1(contacts.at(3));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj2_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(125));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj3_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 3);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(161));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    checkProcessInterpolatedResultsNoTime1(contacts.at(1));
    checkProcessInterpolatedResults(contacts);
  }

  {  // CollisionEvaluatorType::LVS_CONTINUOUS && CollisionCheckProgramType::ALL_EXCEPT_END
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS;
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_END;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(contacts.at(2).size(), 3);
    EXPECT_EQ(contacts.at(3).size(), 2);
    /**< @todo we are getting 9 instead of 6 because duplicate contacts at states */
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(288));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    checkProcessInterpolatedResultsNoTime1(contacts.at(3));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj2_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(123));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    checkProcessInterpolatedResultsNoTime1(contacts.at(1));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj3_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 3);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(161));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    checkProcessInterpolatedResultsNoTime1(contacts.at(0));
    checkProcessInterpolatedResults(contacts);
  }

  {  // CollisionEvaluatorType::LVS_CONTINUOUS && CollisionCheckProgramType::ALL_EXCEPT_START
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS;
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_START;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(contacts.at(2).size(), 3);
    EXPECT_EQ(contacts.at(3).size(), 2);
    /**< @todo we are getting 7 instead of 4 because duplicate contacts at states */
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(288));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    checkProcessInterpolatedResultsNoTime1(contacts.at(3));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj2_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(125));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    checkProcessInterpolatedResultsNoTime1(contacts.at(1));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj3_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 3);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(159));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    checkProcessInterpolatedResultsNoTime1(contacts.at(1));
    checkProcessInterpolatedResults(contacts);
  }

  {  // CollisionEvaluatorType::LVS_CONTINUOUS && CollisionCheckProgramType::INTERMEDIATE_ONLY
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS;
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(contacts.at(2).size(), 3);
    EXPECT_EQ(contacts.at(3).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(288));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    checkProcessInterpolatedResultsNoTime1(contacts.at(3));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj2_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(123));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    checkProcessInterpolatedResultsNoTime1(contacts.at(1));
    checkProcessInterpolatedResults(contacts);

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), static_cast<std::size_t>(traj3_ci.size() - 1));
    EXPECT_EQ(contacts.at(0).size(), 3);
    EXPECT_EQ(contacts.at(1).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(159));
    checkProcessInterpolatedResultsNoTime0(contacts.at(0));
    checkProcessInterpolatedResultsNoTime1(contacts.at(1));
    checkProcessInterpolatedResults(contacts);
  }

  {  // CollisionEvaluatorType::LVS_CONTINUOUS && CollisionCheckProgramType::END_ONLY
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS;
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::END_ONLY;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_FALSE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), 1);
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(0));

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), 1);
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(2));

    contacts.clear();
    EXPECT_FALSE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), 1);
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(0));
  }

  {  // CollisionEvaluatorType::LVS_CONTINUOUS && CollisionCheckProgramType::START_ONLY
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS;
    config.check_program_mode = tesseract_collision::CollisionCheckProgramType::START_ONLY;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    EXPECT_FALSE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj_ci, config));
    EXPECT_EQ(contacts.size(), 1);
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(0));

    contacts.clear();
    EXPECT_FALSE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj2_ci, config));
    EXPECT_EQ(contacts.size(), 1);
    EXPECT_EQ(contacts.at(0).size(), 0);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(0));

    contacts.clear();
    EXPECT_TRUE(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj3_ci, config));
    EXPECT_EQ(contacts.size(), 1);
    EXPECT_EQ(contacts.at(0).size(), 2);
    EXPECT_EQ(getContactCount(contacts), static_cast<int>(2));
  }

  // Failures
  {
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::CONTINUOUS;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(contactCheckProgram(contacts, *discrete_manager, *state_solver, traj_ci, config));
  }
  {
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::DISCRETE;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(contactCheckProgram(
        contacts, *discrete_manager, *state_solver, tesseract_planning::CompositeInstruction(), config));
  }
  {
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::DISCRETE;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(contactCheckProgram(contacts, *continuous_manager, *state_solver, traj_ci, config));
  }
  {
    tesseract_collision::CollisionCheckConfig config;
    config.type = tesseract_collision::CollisionEvaluatorType::CONTINUOUS;
    std::vector<tesseract_collision::ContactResultMap> contacts;
    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(contactCheckProgram(
        contacts, *continuous_manager, *state_solver, tesseract_planning::CompositeInstruction(), config));
  }
}
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
