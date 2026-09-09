#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <filesystem>
#include <string>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/resource_locator.h>
#include <tesseract/common/types.h>
#include <tesseract/common/utils.h>
#include <tesseract/collision/types.h>
#include <tesseract/collision/continuous_contact_manager.h>
#include <tesseract/collision/discrete_contact_manager.h>
#include <tesseract/command_language/composite_instruction.h>
#include <tesseract/command_language/move_instruction.h>
#include <tesseract/command_language/state_waypoint.h>
#include <tesseract/command_language/poly/move_instruction_poly.h>
#include <tesseract/environment/environment.h>
#include <tesseract/environment/commands/add_link_command.h>
#include <tesseract/geometry/impl/sphere.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/scene_graph/link.h>
#include <tesseract/scene_graph/joint.h>
#include <tesseract/state_solver/state_solver.h>
#include <tesseract/motion_planners/utils.h>

using namespace tesseract::collision;
using namespace tesseract::command_language;
using namespace tesseract::environment;
using namespace tesseract::scene_graph;

namespace
{
/** @brief How many link entries came back under each continuous collision type */
struct CcTypeCounts
{
  int none{ 0 };
  int time0{ 0 };
  int time1{ 0 };
  int between{ 0 };

  std::string str() const
  {
    return "none=" + std::to_string(none) + " time0=" + std::to_string(time0) + " time1=" + std::to_string(time1) +
           " between=" + std::to_string(between);
  }
};

/**
 * @brief Assert the guarantees the cc_type/cc_time pair carries, and count the segment ends.
 *
 * The type fixes the time, not the reverse. A contact typed Time0 sits at the start of the segment
 * and carries cc_time 0; one typed Time1 sits at the end and carries cc_time 1; a link that took no
 * part in the cast is typed None and keeps a negative time. The converse must not be asserted: the
 * backend chooses Time0 or Time1 over Between from a support-function comparison with its own
 * tolerance, so a Between contact may legitimately carry a time arbitrarily close to 0 or 1.
 *
 * @return A count of the link entries under each type, so a failure says what did come back
 */
CcTypeCounts checkCcTypeInvariants(const std::vector<ContactResultMap>& contacts)
{
  CcTypeCounts counts;
  for (const auto& c : contacts)
  {
    for (const auto& pair : c)
    {
      for (const auto& r : pair.second)
      {
        for (std::size_t j = 0; j < 2; ++j)
        {
          switch (r.cc_type[j])
          {
            case ContinuousCollisionType::CCType_Time0:
              EXPECT_TRUE(tesseract::common::almostEqualRelativeAndAbs(r.cc_time[j], 0.0));
              ++counts.time0;
              break;
            case ContinuousCollisionType::CCType_Time1:
              EXPECT_TRUE(tesseract::common::almostEqualRelativeAndAbs(r.cc_time[j], 1.0));
              ++counts.time1;
              break;
            case ContinuousCollisionType::CCType_Between:
              EXPECT_GE(r.cc_time[j], 0.0);
              EXPECT_LE(r.cc_time[j], 1.0);
              ++counts.between;
              break;
            case ContinuousCollisionType::CCType_None:
              EXPECT_LT(r.cc_time[j], 0.0);
              ++counts.none;
              break;
          }
        }
      }
    }
  }
  return counts;
}
}  // namespace

class ContactCheckProgramTest : public testing::Test
{
public:
  Environment::Ptr env = std::make_shared<Environment>();
  std::vector<tesseract::common::JointId> joint_ids{ "joint_a1", "joint_a2", "joint_a3", "joint_a4",
                                                     "joint_a5", "joint_a6", "joint_a7" };

  void SetUp() override
  {
    const auto locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
    const std::filesystem::path urdf(
        locator->locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf")->getFilePath());
    const std::filesystem::path srdf(
        locator->locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.srdf")->getFilePath());
    ASSERT_TRUE(env->init(urdf, srdf, locator));

    // Put a sphere in front of the arm so the sweep ends inside it
    Link link_sphere("sphere_attached");
    auto collision = std::make_shared<Collision>();
    collision->origin = Eigen::Isometry3d::Identity();
    collision->origin.translation() = Eigen::Vector3d(0.5, 0, 0.55);
    collision->geometry = std::make_shared<tesseract::geometry::Sphere>(0.15);
    link_sphere.collision.push_back(collision);

    Joint joint_sphere("joint_sphere_attached");
    joint_sphere.parent_link_id = "base_link";
    joint_sphere.child_link_id = link_sphere.getName();
    joint_sphere.type = JointType::FIXED;

    ASSERT_TRUE(env->applyCommand(std::make_shared<AddLinkCommand>(link_sphere, joint_sphere)));
  }

  /**
   * @brief A two-waypoint program whose second waypoint sits inside the sphere
   *
   * The end value is bounded on both sides, not just past the sphere's surface: the continuous
   * manager types a contact Time1 only when its own support-function comparison places the deepest
   * approach at the far end of the cast, and overshooting into the sphere moves that point earlier
   * instead. Measured at longest_valid_segment_length 0.05 from this start, a Time1 comes back for
   * a joint_a1 end value in [-0.33, -0.125]; -0.23 is central, with ~0.1 rad of margin either side.
   */
  CompositeInstruction collidingProgram() const
  {
    Eigen::VectorXd start(7);
    start << -0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0;
    Eigen::VectorXd collision(7);
    collision << -0.23, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0;

    CompositeInstruction program;
    program.push_back(
        MoveInstructionPoly(MoveInstruction(StateWaypoint(joint_ids, start), MoveInstructionType::FREESPACE)));
    program.push_back(
        MoveInstructionPoly(MoveInstruction(StateWaypoint(joint_ids, collision), MoveInstructionType::FREESPACE)));
    return program;
  }

  /**
   * @brief A one-waypoint program whose only state is inside the sphere
   *
   * That state is both the program's start and its end, so it exercises the tie between them.
   */
  CompositeInstruction singleStateCollidingProgram() const
  {
    Eigen::VectorXd position(7);
    position << 0.0, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0;

    CompositeInstruction program;
    program.push_back(
        MoveInstructionPoly(MoveInstruction(StateWaypoint(joint_ids, position), MoveInstructionType::FREESPACE)));
    return program;
  }

  /**
   * @brief A three-waypoint program in which every waypoint's own state is inside the sphere
   *
   * The discrete checks look at states, not sweeps, so each waypoint has to collide on its own.
   * Measured from this arm pose, a single state contacts the sphere for a joint_a1 value in
   * [-0.33, 0.33]; the three waypoints span the middle of that window with at least 0.18 rad of
   * margin at either end.
   */
  CompositeInstruction discreteCollidingProgram() const
  {
    CompositeInstruction program;
    for (const double a1 : { -0.15, 0.0, 0.15 })
    {
      Eigen::VectorXd position(7);
      position << a1, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0;
      program.push_back(
          MoveInstructionPoly(MoveInstruction(StateWaypoint(joint_ids, position), MoveInstructionType::FREESPACE)));
    }
    return program;
  }
};

// A subdivided segment ends at its last cast, so a contact there is at cc_time 1 and must be
// typed Time1. The segment length is short enough to force subdivision.
TEST_F(ContactCheckProgramTest, LvsContinuousSegmentEndIsTime1)  // NOLINT
{
  const auto state_solver = env->getStateSolver();
  const auto manager = env->getContinuousContactManager();
  manager->setActiveCollisionObjects(env->getJointGroup("manipulator")->getActiveLinkIds());

  CollisionCheckConfig config;
  config.type = CollisionEvaluatorType::LVS_CONTINUOUS;
  config.longest_valid_segment_length = 0.05;
  config.check_program_mode = CollisionCheckProgramType::ALL;
  config.exit_condition = CollisionCheckExitType::ALL;

  std::vector<ContactResultMap> contacts;
  EXPECT_TRUE(
      tesseract::motion_planners::contactCheckProgram(contacts, *manager, *state_solver, collidingProgram(), config));

  ASSERT_FALSE(contacts.empty());
  const CcTypeCounts counts = checkCcTypeInvariants(contacts);
  EXPECT_GT(counts.time1, 0) << "Nothing was typed Time1, so the arm marking the end of a subdivided segment never "
                                "fired. Types seen: "
                             << counts.str();
}

// END_ONLY checks the trajectory's last state by definition of the mode, so a contact there is at
// cc_time 1 and must be typed Time1 rather than Time0.
TEST_F(ContactCheckProgramTest, DiscreteEndOnlyIsTime1)  // NOLINT
{
  const auto state_solver = env->getStateSolver();
  const auto manager = env->getDiscreteContactManager();
  manager->setActiveCollisionObjects(env->getJointGroup("manipulator")->getActiveLinkIds());

  CollisionCheckConfig config;
  config.type = CollisionEvaluatorType::DISCRETE;
  config.check_program_mode = CollisionCheckProgramType::END_ONLY;
  config.exit_condition = CollisionCheckExitType::ALL;

  std::vector<ContactResultMap> contacts;
  EXPECT_TRUE(tesseract::motion_planners::contactCheckProgram(
      contacts, *manager, *state_solver, discreteCollidingProgram(), config));

  ASSERT_FALSE(contacts.empty());
  const CcTypeCounts counts = checkCcTypeInvariants(contacts);
  EXPECT_GT(counts.time1, 0) << "The only state END_ONLY checks is the trajectory's end. Types seen: " << counts.str();
  EXPECT_EQ(counts.time0, 0) << "Types seen: " << counts.str();
}

// The discrete walk visits every state; only the last one is the trajectory's end, so it alone is
// typed Time1 and the states before it stay Time0.
TEST_F(ContactCheckProgramTest, DiscreteWalkEndStateIsTime1)  // NOLINT
{
  const auto state_solver = env->getStateSolver();
  const auto manager = env->getDiscreteContactManager();
  manager->setActiveCollisionObjects(env->getJointGroup("manipulator")->getActiveLinkIds());

  CollisionCheckConfig config;
  config.type = CollisionEvaluatorType::DISCRETE;
  config.check_program_mode = CollisionCheckProgramType::ALL;
  config.exit_condition = CollisionCheckExitType::ALL;

  std::vector<ContactResultMap> contacts;
  EXPECT_TRUE(tesseract::motion_planners::contactCheckProgram(
      contacts, *manager, *state_solver, discreteCollidingProgram(), config));

  ASSERT_FALSE(contacts.empty());
  const CcTypeCounts counts = checkCcTypeInvariants(contacts);
  EXPECT_GT(counts.time1, 0) << "The trajectory's last state was not typed Time1. Types seen: " << counts.str();
  EXPECT_GT(counts.time0, 0) << "The states before the end must stay Time0. Types seen: " << counts.str();
}

// ALL_EXCEPT_END stops the walk one state early, so the last state it visits is not the
// trajectory's end and nothing may be typed Time1.
TEST_F(ContactCheckProgramTest, LvsContinuousTrimmedEndIsNotTime1)  // NOLINT
{
  const auto state_solver = env->getStateSolver();
  const auto manager = env->getContinuousContactManager();
  manager->setActiveCollisionObjects(env->getJointGroup("manipulator")->getActiveLinkIds());

  CollisionCheckConfig config;
  config.type = CollisionEvaluatorType::LVS_CONTINUOUS;
  config.longest_valid_segment_length = 0.05;
  config.check_program_mode = CollisionCheckProgramType::ALL_EXCEPT_END;
  config.exit_condition = CollisionCheckExitType::ALL;

  std::vector<ContactResultMap> contacts;
  EXPECT_TRUE(
      tesseract::motion_planners::contactCheckProgram(contacts, *manager, *state_solver, collidingProgram(), config));

  ASSERT_FALSE(contacts.empty());
  const CcTypeCounts counts = checkCcTypeInvariants(contacts);
  EXPECT_GT(counts.between, 0) << "Types seen: " << counts.str();
  EXPECT_EQ(counts.time1, 0) << "The trim stops the walk one cast short of the segment end, so the last cast it "
                                "visits is not that end. Types seen: "
                             << counts.str();
}

TEST_F(ContactCheckProgramTest, DiscreteWalkTrimmedEndIsNotTime1)  // NOLINT
{
  const auto state_solver = env->getStateSolver();
  const auto manager = env->getDiscreteContactManager();
  manager->setActiveCollisionObjects(env->getJointGroup("manipulator")->getActiveLinkIds());

  CollisionCheckConfig config;
  config.type = CollisionEvaluatorType::DISCRETE;
  config.check_program_mode = CollisionCheckProgramType::ALL_EXCEPT_END;
  config.exit_condition = CollisionCheckExitType::ALL;

  std::vector<ContactResultMap> contacts;
  EXPECT_TRUE(tesseract::motion_planners::contactCheckProgram(
      contacts, *manager, *state_solver, discreteCollidingProgram(), config));

  ASSERT_FALSE(contacts.empty());
  const CcTypeCounts counts = checkCcTypeInvariants(contacts);
  EXPECT_GT(counts.time0, 0) << "Types seen: " << counts.str();
  EXPECT_EQ(counts.time1, 0) << "A state visited last only because the walk was trimmed is not the trajectory's end. "
                                "Types seen: "
                             << counts.str();
}

// A continuous manager given one state twice tests a zero length cast, so the contact it reports is
// a point in time rather than a sweep. END_ONLY checks the program's last state, so that point is
// cc_time 1 and the type is Time1 -- never Between, which would claim the contact happened somewhere
// strictly inside an interval that does not exist.
TEST_F(ContactCheckProgramTest, ContinuousEndOnlyIsTime1)  // NOLINT
{
  const auto state_solver = env->getStateSolver();
  const auto manager = env->getContinuousContactManager();
  manager->setActiveCollisionObjects(env->getJointGroup("manipulator")->getActiveLinkIds());

  CollisionCheckConfig config;
  config.type = CollisionEvaluatorType::CONTINUOUS;
  config.check_program_mode = CollisionCheckProgramType::END_ONLY;
  config.exit_condition = CollisionCheckExitType::ALL;

  std::vector<ContactResultMap> contacts;
  EXPECT_TRUE(tesseract::motion_planners::contactCheckProgram(
      contacts, *manager, *state_solver, discreteCollidingProgram(), config));

  ASSERT_FALSE(contacts.empty());
  const CcTypeCounts counts = checkCcTypeInvariants(contacts);
  EXPECT_GT(counts.time1, 0) << "The only state END_ONLY checks is the program's end. Types seen: " << counts.str();
  EXPECT_EQ(counts.time0, 0) << "Types seen: " << counts.str();
  EXPECT_EQ(counts.between, 0) << "A single state check spans no interval, so nothing can be between. Types seen: "
                               << counts.str();
}

// The START_ONLY counterpart: the program's first state, so cc_time 0 and Time0.
TEST_F(ContactCheckProgramTest, ContinuousStartOnlyIsTime0)  // NOLINT
{
  const auto state_solver = env->getStateSolver();
  const auto manager = env->getContinuousContactManager();
  manager->setActiveCollisionObjects(env->getJointGroup("manipulator")->getActiveLinkIds());

  CollisionCheckConfig config;
  config.type = CollisionEvaluatorType::CONTINUOUS;
  config.check_program_mode = CollisionCheckProgramType::START_ONLY;
  config.exit_condition = CollisionCheckExitType::ALL;

  std::vector<ContactResultMap> contacts;
  EXPECT_TRUE(tesseract::motion_planners::contactCheckProgram(
      contacts, *manager, *state_solver, discreteCollidingProgram(), config));

  ASSERT_FALSE(contacts.empty());
  const CcTypeCounts counts = checkCcTypeInvariants(contacts);
  EXPECT_GT(counts.time0, 0) << "The only state START_ONLY checks is the program's start. Types seen: " << counts.str();
  EXPECT_EQ(counts.time1, 0) << "Types seen: " << counts.str();
  EXPECT_EQ(counts.between, 0) << "A single state check spans no interval, so nothing can be between. Types seen: "
                               << counts.str();
}

// A one-waypoint program's only state is both its start and its end. The start wins the tie, so the
// contact is typed Time0 at cc_time 0 rather than Time1.
TEST_F(ContactCheckProgramTest, DiscreteSingleStateIsTime0)  // NOLINT
{
  const auto state_solver = env->getStateSolver();
  const auto manager = env->getDiscreteContactManager();
  manager->setActiveCollisionObjects(env->getJointGroup("manipulator")->getActiveLinkIds());

  CollisionCheckConfig config;
  config.type = CollisionEvaluatorType::DISCRETE;
  config.check_program_mode = CollisionCheckProgramType::ALL;
  config.exit_condition = CollisionCheckExitType::ALL;

  std::vector<ContactResultMap> contacts;
  EXPECT_TRUE(tesseract::motion_planners::contactCheckProgram(
      contacts, *manager, *state_solver, singleStateCollidingProgram(), config));

  ASSERT_FALSE(contacts.empty());
  const CcTypeCounts counts = checkCcTypeInvariants(contacts);
  EXPECT_GT(counts.time0, 0) << "Types seen: " << counts.str();
  EXPECT_EQ(counts.time1, 0) << "A state that is both the start and the end is typed as the start. Types seen: "
                             << counts.str();
}

// The LVS discrete walk subdivides each segment and steps over the sub-states. On the last segment it
// runs one further than on the others, so the program's final state is reached by the walk itself and
// typed Time1 -- no separate end-state check is involved.
TEST_F(ContactCheckProgramTest, LvsDiscreteWalkEndStateIsTime1)  // NOLINT
{
  const auto state_solver = env->getStateSolver();
  const auto manager = env->getDiscreteContactManager();
  manager->setActiveCollisionObjects(env->getJointGroup("manipulator")->getActiveLinkIds());

  CollisionCheckConfig config;
  config.type = CollisionEvaluatorType::LVS_DISCRETE;
  config.longest_valid_segment_length = 0.05;
  config.check_program_mode = CollisionCheckProgramType::ALL;
  config.exit_condition = CollisionCheckExitType::ALL;

  std::vector<ContactResultMap> contacts;
  EXPECT_TRUE(tesseract::motion_planners::contactCheckProgram(
      contacts, *manager, *state_solver, discreteCollidingProgram(), config));

  ASSERT_FALSE(contacts.empty());
  const CcTypeCounts counts = checkCcTypeInvariants(contacts);
  EXPECT_GT(counts.time1, 0) << "The walk reaches the program's final state on the last segment. Types seen: "
                             << counts.str();
  EXPECT_GT(counts.time0, 0) << "Types seen: " << counts.str();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
