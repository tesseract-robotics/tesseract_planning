#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>
#include <tesseract_environment/environment.h>

#include <tesseract_motion_planners/core/types.h>

#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_assign_plan_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_plan_profile.h>

#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/interface_utils.h>

#include <tesseract_task_composer/task_composer_input.h>
#include <tesseract_task_composer/task_composer_data_storage.h>
#include <tesseract_task_composer/task_composer_utils.h>
#include <tesseract_task_composer/nodes/min_length_task.h>
#include <tesseract_task_composer/nodes/raster_ft_global_pipeline_task.h>
#include <tesseract_task_composer/nodes/raster_ft_motion_task.h>
#include <tesseract_task_composer/nodes/raster_ft_only_global_pipeline_task.h>
#include <tesseract_task_composer/nodes/raster_ft_only_motion_task.h>
#include <tesseract_task_composer/profiles/min_length_profile.h>
#include <tesseract_task_composer/taskflow/taskflow_task_composer_executor.h>

#include <tesseract_support/tesseract_support_resource_locator.h>

#include "raster_example_program.h"
#include "freespace_example_program.h"

using namespace tesseract_kinematics;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_planning;
using namespace tesseract_planning::node_names;
using namespace tesseract_planning::profile_ns;

class TesseractTaskComposerUnit : public ::testing::Test
{
protected:
  Environment::Ptr env_;
  ManipulatorInfo manip;

  void SetUp() override
  {
    auto locator = std::make_shared<tesseract_common::TesseractSupportResourceLocator>();
    Environment::Ptr env = std::make_shared<Environment>();
    tesseract_common::fs::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400.urdf");
    tesseract_common::fs::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400.srdf");
    EXPECT_TRUE(env->init(urdf_path, srdf_path, locator));
    env_ = env;

    manip.manipulator = "manipulator";
    manip.manipulator_ik_solver = "OPWInvKin";
    manip.working_frame = "base_link";
    manip.tcp_frame = "tool0";
  }
};

TEST_F(TesseractTaskComposerUnit, MinLengthTaskTest)  // NOLINT
{
  tesseract_planning::CompositeInstruction program = freespaceExampleProgramABB();
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  program.setManipulatorInfo(manip);
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  // Define the Process Input
  auto cur_state = env_->getState();
  CompositeInstruction interpolated_program = generateInterpolatedProgram(program, cur_state, env_);

  TaskComposerDataStorage task_data;
  task_data.setData("input_program", interpolated_program);

  TaskComposerProblem task_problem(env_, task_data);

  auto profiles = std::make_shared<ProfileDictionary>();
  long current_length = interpolated_program.getMoveInstructionCount();
  profiles->addProfile<MinLengthProfile>(
      MIN_LENGTH_TASK_NAME, program.getProfile(), std::make_shared<MinLengthProfile>(current_length));

  auto task_input = std::make_shared<TaskComposerInput>(task_problem, profiles);

  MinLengthTask task("input_program", "output_program", true);
  EXPECT_TRUE(task.run(*task_input) == 1);
  long final_length =
      task_input->data_storage.getData("output_program").as<CompositeInstruction>().getMoveInstructionCount();
  EXPECT_TRUE(final_length == current_length);

  profiles->addProfile<MinLengthProfile>(
      MIN_LENGTH_TASK_NAME, program.getProfile(), std::make_shared<MinLengthProfile>(2 * current_length));

  task_input->reset();
  EXPECT_TRUE(task.run(*task_input) == 1);
  long final_length2 =
      task_input->data_storage.getData("output_program").as<CompositeInstruction>().getMoveInstructionCount();
  EXPECT_TRUE(final_length2 >= (2 * current_length));

  profiles->addProfile<MinLengthProfile>(
      MIN_LENGTH_TASK_NAME, program.getProfile(), std::make_shared<MinLengthProfile>(3 * current_length));

  task_input->reset();
  EXPECT_TRUE(task.run(*task_input) == 1);
  long final_length3 =
      task_input->data_storage.getData("output_program").as<CompositeInstruction>().getMoveInstructionCount();
  EXPECT_TRUE(final_length3 >= (3 * current_length));
}

TEST_F(TesseractTaskComposerUnit, RasterSimpleMotionPlannerFixedSizeAssignPlanProfileTest)  // NOLINT
{
  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  tesseract_planning::CompositeInstruction program = rasterExampleProgram();
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  program.setManipulatorInfo(manip);
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  // Profile Dictionary
  auto profiles = std::make_shared<ProfileDictionary>();
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, process_profile, std::make_shared<SimplePlannerFixedSizeAssignPlanProfile>());
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, freespace_profile, std::make_shared<SimplePlannerFixedSizeAssignPlanProfile>());

  auto interpolator = std::make_shared<SimpleMotionPlanner>();

  // Create Planning Request
  PlannerRequest request;
  request.instructions = program;
  request.env = env_;
  request.env_state = env_->getState();
  request.profiles = profiles;

  PlannerResponse response = interpolator->solve(request);
  EXPECT_TRUE(response);

  auto pcnt = request.instructions.getMoveInstructionCount();
  auto mcnt = response.results.getMoveInstructionCount();

  // The first plan instruction is the start instruction and every other plan instruction should be converted into
  // ten move instruction.
  EXPECT_EQ(((pcnt - 1) * 10) + 1, mcnt);
  EXPECT_FALSE(response.results.getManipulatorInfo().empty());
}

TEST_F(TesseractTaskComposerUnit, RasterSimpleMotionPlannerLVSPlanProfileTest)  // NOLINT
{
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  tesseract_planning::CompositeInstruction program = rasterExampleProgram();
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  program.setManipulatorInfo(manip);
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  // Profile Dictionary
  auto profiles = std::make_shared<ProfileDictionary>();
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, process_profile, std::make_shared<SimplePlannerLVSPlanProfile>());
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, freespace_profile, std::make_shared<SimplePlannerLVSPlanProfile>());

  auto interpolator = std::make_shared<SimpleMotionPlanner>();

  // Create Planning Request
  PlannerRequest request;
  request.instructions = program;
  request.env = env_;
  request.env_state = env_->getState();
  request.profiles = profiles;

  PlannerResponse response = interpolator->solve(request);
  EXPECT_TRUE(response);

  auto mcnt = response.results.getMoveInstructionCount();

  // The first plan instruction is the start instruction and every other plan instruction should be converted into
  // ten move instruction.
  EXPECT_EQ(98, mcnt);
  EXPECT_FALSE(response.results.getManipulatorInfo().empty());
}

TEST_F(TesseractTaskComposerUnit, RasterSimpleMotionPlannerDefaultLVSNoIKPlanProfileTest)  // NOLINT
{
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  tesseract_planning::CompositeInstruction program = rasterExampleProgram();
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  program.setManipulatorInfo(manip);
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  // Profile Dictionary
  auto profiles = std::make_shared<ProfileDictionary>();
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, process_profile, std::make_shared<SimplePlannerLVSNoIKPlanProfile>());
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, freespace_profile, std::make_shared<SimplePlannerLVSNoIKPlanProfile>());

  auto interpolator = std::make_shared<SimpleMotionPlanner>();

  // Create Planning Request
  PlannerRequest request;
  request.instructions = program;
  request.env = env_;
  request.env_state = env_->getState();
  request.profiles = profiles;

  PlannerResponse response = interpolator->solve(request);
  EXPECT_TRUE(response);

  auto mcnt = response.results.getMoveInstructionCount();

  // The first plan instruction is the start instruction and every other plan instruction should be converted into
  // ten move instruction.
  EXPECT_EQ(83, mcnt);
  EXPECT_FALSE(response.results.getManipulatorInfo().empty());
}

TEST_F(TesseractTaskComposerUnit, FreespaceSimpleMotionPlannerFixedSizeAssignPlanProfileTest)  // NOLINT
{
  CompositeInstruction program = freespaceExampleProgramABB();
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  program.setManipulatorInfo(manip);
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  auto interpolator = std::make_shared<SimpleMotionPlanner>();

  // Profile Dictionary
  auto profiles = std::make_shared<ProfileDictionary>();
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, DEFAULT_PROFILE_KEY, std::make_shared<SimplePlannerFixedSizeAssignPlanProfile>());

  // Create Planning Request
  PlannerRequest request;
  request.instructions = program;
  request.env = env_;
  request.env_state = env_->getState();
  request.profiles = profiles;

  PlannerResponse response = interpolator->solve(request);
  EXPECT_TRUE(response);

  auto pcnt = request.instructions.getMoveInstructionCount();
  auto mcnt = response.results.getMoveInstructionCount();

  // The first plan instruction is the start instruction and every other plan instruction should be converted into
  // ten move instruction.
  EXPECT_EQ(((pcnt - 1) * 10) + 1, mcnt);
  EXPECT_FALSE(response.results.getManipulatorInfo().empty());
}

TEST_F(TesseractTaskComposerUnit, FreespaceSimpleMotionPlannerDefaultLVSPlanProfileTest)  // NOLINT
{
  CompositeInstruction program = freespaceExampleProgramABB();
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  program.setManipulatorInfo(manip);
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  // Profile Dictionary
  auto profiles = std::make_shared<ProfileDictionary>();
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, DEFAULT_PROFILE_KEY, std::make_shared<SimplePlannerLVSPlanProfile>());

  auto interpolator = std::make_shared<SimpleMotionPlanner>();

  // Create Planning Request
  PlannerRequest request;
  request.instructions = program;
  request.env = env_;
  request.env_state = env_->getState();
  request.profiles = profiles;

  PlannerResponse response = interpolator->solve(request);
  EXPECT_TRUE(response);

  auto mcnt = response.results.getMoveInstructionCount();

  // The first plan instruction is the start instruction and every other plan instruction should be converted into
  // 32 move instruction.
  EXPECT_EQ(37, mcnt);
  EXPECT_FALSE(response.results.getManipulatorInfo().empty());
}

TEST_F(TesseractTaskComposerUnit, RasterProcessManagerDefaultPlanProfileTest)  // NOLINT
{
  // Create Executor
  auto executor = std::make_unique<TaskflowTaskComposerExecutor>();

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterExampleProgram(freespace_profile, process_profile);

  // Add profiles to planning server
  auto profiles = std::make_shared<ProfileDictionary>();
  addDefaultPlannerProfiles(*profiles, { freespace_profile, process_profile });
  addDefaultTaskComposerProfiles(*profiles, { freespace_profile, process_profile });

  auto simple_plan_profile = std::make_shared<SimplePlannerFixedSizeAssignPlanProfile>();
  profiles->addProfile<SimplePlannerPlanProfile>(SIMPLE_DEFAULT_NAMESPACE, freespace_profile, simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(SIMPLE_DEFAULT_NAMESPACE, process_profile, simple_plan_profile);

  // Create data storage
  TaskComposerDataStorage task_data;
  task_data.setData("input_program", program);

  // Create problem
  TaskComposerProblem task_problem(env_, task_data);

  // Create input
  auto task_input = std::make_shared<TaskComposerInput>(task_problem, profiles);

  // Create task
  RasterFtMotionTask task("input_program", "output_program");

  // Solve task
  TaskComposerFuture::UPtr future = executor->run(task, *task_input);
  future->wait();

  // Confirm that the task is finished
  EXPECT_TRUE(future->ready());

  // Solve
  EXPECT_TRUE(task_input->isSuccessful());
}

TEST_F(TesseractTaskComposerUnit, RasterGlobalProcessManagerDefaultPlanProfileTest)  // NOLINT
{
  // Create Executor
  auto executor = std::make_unique<TaskflowTaskComposerExecutor>();

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterExampleProgram(freespace_profile, process_profile);

  // Add profiles to planning server
  auto profiles = std::make_shared<ProfileDictionary>();
  addDefaultPlannerProfiles(*profiles, { freespace_profile, process_profile });
  addDefaultTaskComposerProfiles(*profiles, { freespace_profile, process_profile });

  auto simple_plan_profile = std::make_shared<SimplePlannerFixedSizeAssignPlanProfile>();
  profiles->addProfile<SimplePlannerPlanProfile>(SIMPLE_DEFAULT_NAMESPACE, freespace_profile, simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(SIMPLE_DEFAULT_NAMESPACE, process_profile, simple_plan_profile);

  // Create data storage
  TaskComposerDataStorage task_data;
  task_data.setData("input_program", program);

  // Create problem
  TaskComposerProblem task_problem(env_, task_data);

  // Create input
  auto task_input = std::make_shared<TaskComposerInput>(task_problem, profiles);

  // Create task
  RasterFtGlobalPipelineTask task("input_program", "output_program");

  // Solve task
  TaskComposerFuture::UPtr future = executor->run(task, *task_input);
  future->wait();

  // Confirm that the task is finished
  EXPECT_TRUE(future->ready());

  // Solve
  EXPECT_TRUE(task_input->isSuccessful());
}

TEST_F(TesseractTaskComposerUnit, RasterGlobalProcessManagerDefaultLVSPlanProfileTest)  // NOLINT
{
  // Create Executor
  auto executor = std::make_unique<TaskflowTaskComposerExecutor>();

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterExampleProgram(freespace_profile, process_profile);

  // Add profiles to planning server
  auto profiles = std::make_shared<ProfileDictionary>();
  addDefaultPlannerProfiles(*profiles, { freespace_profile, process_profile });
  addDefaultTaskComposerProfiles(*profiles, { freespace_profile, process_profile });

  auto lvs_simple_plan_profile = std::make_shared<SimplePlannerLVSPlanProfile>();
  profiles->addProfile<SimplePlannerPlanProfile>(SIMPLE_DEFAULT_NAMESPACE, freespace_profile, lvs_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(SIMPLE_DEFAULT_NAMESPACE, process_profile, lvs_simple_plan_profile);

  // Create data storage
  TaskComposerDataStorage task_data;
  task_data.setData("input_program", program);

  // Create problem
  TaskComposerProblem task_problem(env_, task_data);

  // Create input
  auto task_input = std::make_shared<TaskComposerInput>(task_problem, profiles);

  // Create task
  RasterFtGlobalPipelineTask task("input_program", "output_program");

  // Solve task
  TaskComposerFuture::UPtr future = executor->run(task, *task_input);
  future->wait();

  // Confirm that the task is finished
  EXPECT_TRUE(future->ready());

  // Solve
  EXPECT_TRUE(task_input->isSuccessful());
}

TEST_F(TesseractTaskComposerUnit, RasterOnlyProcessManagerDefaultPlanProfileTest)  // NOLINT
{
  // Create Executor
  auto executor = std::make_unique<TaskflowTaskComposerExecutor>();

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterOnlyExampleProgram(freespace_profile, process_profile);

  // Add profiles to planning server
  auto profiles = std::make_shared<ProfileDictionary>();
  addDefaultPlannerProfiles(*profiles, { freespace_profile, process_profile });
  addDefaultTaskComposerProfiles(*profiles, { freespace_profile, process_profile });

  auto simple_plan_profile = std::make_shared<SimplePlannerFixedSizeAssignPlanProfile>();
  profiles->addProfile<SimplePlannerPlanProfile>(SIMPLE_DEFAULT_NAMESPACE, freespace_profile, simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(SIMPLE_DEFAULT_NAMESPACE, process_profile, simple_plan_profile);

  // Create data storage
  TaskComposerDataStorage task_data;
  task_data.setData("input_program", program);

  // Create problem
  TaskComposerProblem task_problem(env_, task_data);

  // Create input
  auto task_input = std::make_shared<TaskComposerInput>(task_problem, profiles);

  // Create task
  RasterFtOnlyMotionTask task("input_program", "output_program");

  // Solve task
  TaskComposerFuture::UPtr future = executor->run(task, *task_input);
  future->wait();

  // Confirm that the task is finished
  EXPECT_TRUE(future->ready());

  // Solve
  EXPECT_TRUE(task_input->isSuccessful());
}

TEST_F(TesseractTaskComposerUnit, RasterOnlyProcessManagerDefaultLVSPlanProfileTest)  // NOLINT
{
  // Create Executor
  auto executor = std::make_unique<TaskflowTaskComposerExecutor>();

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterOnlyExampleProgram(freespace_profile, process_profile);

  // Add profiles to planning server
  auto profiles = std::make_shared<ProfileDictionary>();
  addDefaultPlannerProfiles(*profiles, { freespace_profile, process_profile });
  addDefaultTaskComposerProfiles(*profiles, { freespace_profile, process_profile });

  auto lvs_simple_plan_profile = std::make_shared<SimplePlannerLVSPlanProfile>();
  profiles->addProfile<SimplePlannerPlanProfile>(SIMPLE_DEFAULT_NAMESPACE, freespace_profile, lvs_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(SIMPLE_DEFAULT_NAMESPACE, process_profile, lvs_simple_plan_profile);

  // Create data storage
  TaskComposerDataStorage task_data;
  task_data.setData("input_program", program);

  // Create problem
  TaskComposerProblem task_problem(env_, task_data);

  // Create input
  auto task_input = std::make_shared<TaskComposerInput>(task_problem, profiles);

  // Create task
  RasterFtOnlyMotionTask task("input_program", "output_program");

  // Solve task
  TaskComposerFuture::UPtr future = executor->run(task, *task_input);
  future->wait();

  // Confirm that the task is finished
  EXPECT_TRUE(future->ready());

  // Solve
  EXPECT_TRUE(task_input->isSuccessful());
}

TEST_F(TesseractTaskComposerUnit, RasterOnlyGlobalProcessManagerDefaultPlanProfileTest)  // NOLINT
{
  // Create Executor
  auto executor = std::make_unique<TaskflowTaskComposerExecutor>();

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterOnlyExampleProgram(freespace_profile, process_profile);

  // Add profiles to planning server
  auto profiles = std::make_shared<ProfileDictionary>();
  addDefaultPlannerProfiles(*profiles, { freespace_profile, process_profile });
  addDefaultTaskComposerProfiles(*profiles, { freespace_profile, process_profile });

  auto simple_plan_profile = std::make_shared<SimplePlannerFixedSizeAssignPlanProfile>();
  profiles->addProfile<SimplePlannerPlanProfile>(SIMPLE_DEFAULT_NAMESPACE, freespace_profile, simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(SIMPLE_DEFAULT_NAMESPACE, process_profile, simple_plan_profile);

  // Create data storage
  TaskComposerDataStorage task_data;
  task_data.setData("input_program", program);

  // Create problem
  TaskComposerProblem task_problem(env_, task_data);

  // Create input
  auto task_input = std::make_shared<TaskComposerInput>(task_problem, profiles);

  // Create task
  RasterFtOnlyGlobalPipelineTask task("input_program", "output_program");

  // Solve task
  TaskComposerFuture::UPtr future = executor->run(task, *task_input);
  future->wait();

  // Confirm that the task is finished
  EXPECT_TRUE(future->ready());

  // Solve
  EXPECT_TRUE(task_input->isSuccessful());
}

TEST_F(TesseractTaskComposerUnit, RasterOnlyGlobalProcessManagerDefaultLVSPlanProfileTest)  // NOLINT
{
  // Create Executor
  auto executor = std::make_unique<TaskflowTaskComposerExecutor>();

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterOnlyExampleProgram(freespace_profile, process_profile);

  // Add profiles to planning server
  auto profiles = std::make_shared<ProfileDictionary>();
  addDefaultPlannerProfiles(*profiles, { freespace_profile, process_profile });
  addDefaultTaskComposerProfiles(*profiles, { freespace_profile, process_profile });

  auto lvs_simple_plan_profile = std::make_shared<SimplePlannerLVSPlanProfile>();
  profiles->addProfile<SimplePlannerPlanProfile>(SIMPLE_DEFAULT_NAMESPACE, freespace_profile, lvs_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(SIMPLE_DEFAULT_NAMESPACE, process_profile, lvs_simple_plan_profile);

  // Create data storage
  TaskComposerDataStorage task_data;
  task_data.setData("input_program", program);

  // Create problem
  TaskComposerProblem task_problem(env_, task_data);

  // Create input
  auto task_input = std::make_shared<TaskComposerInput>(task_problem, profiles);

  // Create task
  RasterFtOnlyGlobalPipelineTask task("input_program", "output_program");

  // Solve task
  TaskComposerFuture::UPtr future = executor->run(task, *task_input);
  future->wait();

  // Confirm that the task is finished
  EXPECT_TRUE(future->ready());

  // Solve
  EXPECT_TRUE(task_input->isSuccessful());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
