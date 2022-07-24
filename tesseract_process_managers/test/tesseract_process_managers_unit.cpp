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

#include <tesseract_process_managers/core/task_input.h>
#include <tesseract_process_managers/core/process_planning_server.h>
#include <tesseract_process_managers/taskflow_generators/raster_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_global_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_only_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_only_global_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_dt_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_waad_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_waad_dt_taskflow.h>
#include <tesseract_process_managers/task_generators/seed_min_length_task_generator.h>
#include <tesseract_process_managers/task_profiles/seed_min_length_profile.h>
#include <tesseract_support/tesseract_support_resource_locator.h>

#include "raster_example_program.h"
#include "raster_dt_example_program.h"
#include "raster_waad_example_program.h"
#include "raster_waad_dt_example_program.h"
#include "freespace_example_program.h"

using namespace tesseract_kinematics;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_planning;
using namespace tesseract_planning::profile_ns;

class TesseractProcessManagerUnit : public ::testing::Test
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

TEST_F(TesseractProcessManagerUnit, SeedMinLengthTaskGeneratorTest)  // NOLINT
{
  tesseract_planning::CompositeInstruction program = freespaceExampleProgramABB();
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  program.setManipulatorInfo(manip);
  EXPECT_TRUE(program.hasStartInstruction());
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  // Define the Process Input
  auto cur_state = env_->getState();
  CompositeInstruction seed = generateSeed(program, cur_state, env_);

  InstructionPoly program_instruction = program;
  InstructionPoly seed_instruction = seed;

  auto profiles = std::make_shared<ProfileDictionary>();
  long current_length = seed.getMoveInstructionCount();
  TaskInput input(env_, &program_instruction, program.getManipulatorInfo(), &seed_instruction, true, profiles);

  profiles->addProfile<SeedMinLengthProfile>(
      SEED_MIN_LENGTH_DEFAULT_NAMESPACE, program.getProfile(), std::make_shared<SeedMinLengthProfile>(current_length));
  SeedMinLengthTaskGenerator smlpg;
  EXPECT_TRUE(smlpg.conditionalProcess(input, 1) == 1);
  long final_length = input.getResults()->as<CompositeInstruction>().getMoveInstructionCount();
  EXPECT_TRUE(final_length == current_length);

  profiles->addProfile<SeedMinLengthProfile>(SEED_MIN_LENGTH_DEFAULT_NAMESPACE,
                                             program.getProfile(),
                                             std::make_shared<SeedMinLengthProfile>(2 * current_length));
  SeedMinLengthTaskGenerator smlpg2;
  EXPECT_TRUE(smlpg2.conditionalProcess(input, 2) == 1);
  long final_length2 = input.getResults()->as<CompositeInstruction>().getMoveInstructionCount();
  EXPECT_TRUE(final_length2 >= (2 * current_length));

  seed_instruction = seed;
  TaskInput input2(env_, &program_instruction, program.getManipulatorInfo(), &seed_instruction, true, profiles);

  profiles->addProfile<SeedMinLengthProfile>(SEED_MIN_LENGTH_DEFAULT_NAMESPACE,
                                             program.getProfile(),
                                             std::make_shared<SeedMinLengthProfile>(3 * current_length));
  SeedMinLengthTaskGenerator smlpg3;
  EXPECT_TRUE(smlpg3.conditionalProcess(input, 3) == 1);
  long final_length3 = input2.getResults()->as<CompositeInstruction>().getMoveInstructionCount();
  EXPECT_TRUE(final_length3 >= (3 * current_length));
}

TEST_F(TesseractProcessManagerUnit, RasterSimpleMotionPlannerFixedSizeAssignPlanProfileTest)  // NOLINT
{
  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  tesseract_planning::CompositeInstruction program = rasterExampleProgram();
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  program.setManipulatorInfo(manip);
  EXPECT_TRUE(program.hasStartInstruction());
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

  PlannerResponse response;
  auto status = interpolator->solve(request, response);
  EXPECT_TRUE(status);

  auto pcnt = request.instructions.getMoveInstructionCount();
  auto mcnt = response.results.getMoveInstructionCount();

  // The first plan instruction is the start instruction and every other plan instruction should be converted into
  // ten move instruction.
  EXPECT_EQ(((pcnt - 1) * 10) + 1, mcnt);
  EXPECT_TRUE(response.results.hasStartInstruction());
  EXPECT_FALSE(response.results.getManipulatorInfo().empty());
}

TEST_F(TesseractProcessManagerUnit, RasterSimpleMotionPlannerLVSPlanProfileTest)  // NOLINT
{
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  tesseract_planning::CompositeInstruction program = rasterExampleProgram();
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  program.setManipulatorInfo(manip);
  EXPECT_TRUE(program.hasStartInstruction());
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

  PlannerResponse response;
  auto status = interpolator->solve(request, response);
  EXPECT_TRUE(status);

  auto mcnt = response.results.getMoveInstructionCount();

  // The first plan instruction is the start instruction and every other plan instruction should be converted into
  // ten move instruction.
  EXPECT_EQ(98, mcnt);
  EXPECT_TRUE(response.results.hasStartInstruction());
  EXPECT_FALSE(response.results.getManipulatorInfo().empty());
}

TEST_F(TesseractProcessManagerUnit, RasterSimpleMotionPlannerDefaultLVSNoIKPlanProfileTest)  // NOLINT
{
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  tesseract_planning::CompositeInstruction program = rasterExampleProgram();
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  program.setManipulatorInfo(manip);
  EXPECT_TRUE(program.hasStartInstruction());
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

  PlannerResponse response;
  auto status = interpolator->solve(request, response);
  EXPECT_TRUE(status);

  auto mcnt = response.results.getMoveInstructionCount();

  // The first plan instruction is the start instruction and every other plan instruction should be converted into
  // ten move instruction.
  EXPECT_EQ(83, mcnt);
  EXPECT_TRUE(response.results.hasStartInstruction());
  EXPECT_FALSE(response.results.getManipulatorInfo().empty());
}

TEST_F(TesseractProcessManagerUnit, FreespaceSimpleMotionPlannerFixedSizeAssignPlanProfileTest)  // NOLINT
{
  CompositeInstruction program = freespaceExampleProgramABB();
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  program.setManipulatorInfo(manip);
  EXPECT_TRUE(program.hasStartInstruction());
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

  PlannerResponse response;
  auto status = interpolator->solve(request, response);
  EXPECT_TRUE(status);

  auto pcnt = request.instructions.getMoveInstructionCount();
  auto mcnt = response.results.getMoveInstructionCount();

  // The first plan instruction is the start instruction and every other plan instruction should be converted into
  // ten move instruction.
  EXPECT_EQ(((pcnt - 1) * 10) + 1, mcnt);
  EXPECT_TRUE(response.results.hasStartInstruction());
  EXPECT_FALSE(response.results.getManipulatorInfo().empty());
}

TEST_F(TesseractProcessManagerUnit, FreespaceSimpleMotionPlannerDefaultLVSPlanProfileTest)  // NOLINT
{
  CompositeInstruction program = freespaceExampleProgramABB();
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  program.setManipulatorInfo(manip);
  EXPECT_TRUE(program.hasStartInstruction());
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

  PlannerResponse response;
  auto status = interpolator->solve(request, response);
  EXPECT_TRUE(status);

  auto mcnt = response.results.getMoveInstructionCount();

  // The first plan instruction is the start instruction and every other plan instruction should be converted into
  // 32 move instruction.
  EXPECT_EQ(37, mcnt);
  EXPECT_TRUE(response.results.hasStartInstruction());
  EXPECT_FALSE(response.results.getManipulatorInfo().empty());
}

TEST_F(TesseractProcessManagerUnit, RasterProcessManagerDefaultPlanProfileTest)  // NOLINT
{
  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env_), 1);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = process_planner_names::RASTER_FT_PLANNER_NAME;

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterExampleProgram(freespace_profile, process_profile);
  request.instructions = InstructionPoly(program);

  // Add profiles to planning server
  auto default_simple_plan_profile = std::make_shared<SimplePlannerFixedSizeAssignPlanProfile>();
  ProfileDictionary::Ptr profiles = planning_server.getProfiles();
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, freespace_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, process_profile, default_simple_plan_profile);

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Confirm that the task is finished
  EXPECT_TRUE(response.ready());

  // Solve
  EXPECT_TRUE(response.interface->isSuccessful());
}

TEST_F(TesseractProcessManagerUnit, RasterProcessManagerDefaultLVSPlanProfileTest)  // NOLINT
{
  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env_), 1);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = process_planner_names::RASTER_FT_PLANNER_NAME;

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterExampleProgram(freespace_profile, process_profile);
  request.instructions = InstructionPoly(program);

  // Add profiles to planning server
  auto default_simple_plan_profile = std::make_shared<SimplePlannerLVSPlanProfile>();
  ProfileDictionary::Ptr profiles = planning_server.getProfiles();
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, freespace_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, process_profile, default_simple_plan_profile);

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Confirm that the task is finished
  EXPECT_TRUE(response.ready());

  // Solve
  EXPECT_TRUE(response.interface->isSuccessful());

  // Test planning server thread safety
#pragma omp parallel for num_threads(10) shared(planning_server, request)
  for (long i = 0; i < 10; ++i)  // NOLINT
  {
    const int tn = omp_get_thread_num();
    CONSOLE_BRIDGE_logDebug("Thread (ID: %i): %i of %i", tn, i, 10);
    ProcessPlanningFuture response = planning_server.run(request);
    response.wait();

    // Confirm that the task is finished and still successful
    EXPECT_TRUE(response.ready());
    EXPECT_TRUE(response.interface->isSuccessful());
  }
}

TEST_F(TesseractProcessManagerUnit, RasterGlobalProcessManagerDefaultPlanProfileTest)  // NOLINT
{
  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env_), 1);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = process_planner_names::RASTER_G_FT_PLANNER_NAME;

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterExampleProgram(freespace_profile, process_profile);
  request.instructions = InstructionPoly(program);

  // Add profiles to planning server
  auto default_simple_plan_profile = std::make_shared<SimplePlannerFixedSizeAssignPlanProfile>();
  ProfileDictionary::Ptr profiles = planning_server.getProfiles();
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, freespace_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, process_profile, default_simple_plan_profile);

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Confirm that the task is finished
  EXPECT_TRUE(response.ready());

  // Solve
  EXPECT_TRUE(response.interface->isSuccessful());
}

TEST_F(TesseractProcessManagerUnit, RasterGlobalProcessManagerDefaultLVSPlanProfileTest)  // NOLINT
{
  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env_), 1);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = process_planner_names::RASTER_G_FT_PLANNER_NAME;

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterExampleProgram(freespace_profile, process_profile);
  request.instructions = InstructionPoly(program);

  // Add profiles to planning server
  auto default_simple_plan_profile = std::make_shared<SimplePlannerLVSPlanProfile>();
  ProfileDictionary::Ptr profiles = planning_server.getProfiles();
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, freespace_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, process_profile, default_simple_plan_profile);

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Confirm that the task is finished
  EXPECT_TRUE(response.ready());

  // Solve
  EXPECT_TRUE(response.interface->isSuccessful());
}

TEST_F(TesseractProcessManagerUnit, RasterOnlyProcessManagerDefaultPlanProfileTest)  // NOLINT
{
  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env_), 1);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = process_planner_names::RASTER_O_FT_PLANNER_NAME;

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterOnlyExampleProgram(freespace_profile, process_profile);
  request.instructions = InstructionPoly(program);

  // Add profiles to planning server
  auto default_simple_plan_profile = std::make_shared<SimplePlannerFixedSizeAssignPlanProfile>();
  ProfileDictionary::Ptr profiles = planning_server.getProfiles();
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, freespace_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, process_profile, default_simple_plan_profile);

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Confirm that the task is finished
  EXPECT_TRUE(response.ready());

  // Solve
  EXPECT_TRUE(response.interface->isSuccessful());
}

TEST_F(TesseractProcessManagerUnit, RasterOnlyProcessManagerDefaultLVSPlanProfileTest)  // NOLINT
{
  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env_), 1);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = process_planner_names::RASTER_O_FT_PLANNER_NAME;

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterOnlyExampleProgram(freespace_profile, process_profile);
  request.instructions = InstructionPoly(program);

  // Add profiles to planning server
  auto default_simple_plan_profile = std::make_shared<SimplePlannerLVSPlanProfile>();
  ProfileDictionary::Ptr profiles = planning_server.getProfiles();
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, freespace_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, process_profile, default_simple_plan_profile);

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Confirm that the task is finished
  EXPECT_TRUE(response.ready());

  // Solve
  EXPECT_TRUE(response.interface->isSuccessful());
}

TEST_F(TesseractProcessManagerUnit, RasterOnlyGlobalProcessManagerDefaultPlanProfileTest)  // NOLINT
{
  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env_), 1);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = process_planner_names::RASTER_O_G_FT_PLANNER_NAME;

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterOnlyExampleProgram(freespace_profile, process_profile);
  request.instructions = InstructionPoly(program);

  // Add profiles to planning server
  auto default_simple_plan_profile = std::make_shared<SimplePlannerFixedSizeAssignPlanProfile>();
  ProfileDictionary::Ptr profiles = planning_server.getProfiles();
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, freespace_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, process_profile, default_simple_plan_profile);

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Confirm that the task is finished
  EXPECT_TRUE(response.ready());

  // Solve
  EXPECT_TRUE(response.interface->isSuccessful());
}

TEST_F(TesseractProcessManagerUnit, RasterOnlyGlobalProcessManagerDefaultLVSPlanProfileTest)  // NOLINT
{
  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env_), 1);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = process_planner_names::RASTER_O_G_FT_PLANNER_NAME;

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterOnlyExampleProgram(freespace_profile, process_profile);
  request.instructions = InstructionPoly(program);

  // Add profiles to planning server
  auto default_simple_plan_profile = std::make_shared<SimplePlannerLVSPlanProfile>();
  ProfileDictionary::Ptr profiles = planning_server.getProfiles();
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, freespace_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, process_profile, default_simple_plan_profile);

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Confirm that the task is finished
  EXPECT_TRUE(response.ready());

  // Solve
  EXPECT_TRUE(response.interface->isSuccessful());
}

TEST_F(TesseractProcessManagerUnit, RasterDTProcessManagerDefaultPlanProfileTest)  // NOLINT
{
  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env_), 1);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = process_planner_names::RASTER_FT_DT_PLANNER_NAME;

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterDTExampleProgram(freespace_profile, process_profile);
  request.instructions = InstructionPoly(program);

  // Add profiles to planning server
  auto default_simple_plan_profile = std::make_shared<SimplePlannerFixedSizeAssignPlanProfile>();
  ProfileDictionary::Ptr profiles = planning_server.getProfiles();
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, freespace_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, process_profile, default_simple_plan_profile);

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Confirm that the task is finished
  EXPECT_TRUE(response.ready());

  // Solve
  EXPECT_TRUE(response.interface->isSuccessful());
}

TEST_F(TesseractProcessManagerUnit, RasterDTProcessManagerDefaultLVSPlanProfileTest)  // NOLINT
{
  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env_), 1);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = process_planner_names::RASTER_FT_DT_PLANNER_NAME;

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterDTExampleProgram(freespace_profile, process_profile);
  request.instructions = InstructionPoly(program);

  // Add profiles to planning server
  auto default_simple_plan_profile = std::make_shared<SimplePlannerLVSPlanProfile>();
  ProfileDictionary::Ptr profiles = planning_server.getProfiles();
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, freespace_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, process_profile, default_simple_plan_profile);

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Confirm that the task is finished
  EXPECT_TRUE(response.ready());

  // Solve
  EXPECT_TRUE(response.interface->isSuccessful());
}

TEST_F(TesseractProcessManagerUnit, RasterWAADProcessManagerDefaultPlanProfileTest)  // NOLINT
{
  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env_), 1);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = process_planner_names::RASTER_FT_WAAD_PLANNER_NAME;

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string approach_profile = "APPROACH";
  std::string process_profile = "PROCESS";
  std::string departure_profile = "DEPARTURE";

  CompositeInstruction program =
      rasterWAADExampleProgram(freespace_profile, approach_profile, process_profile, departure_profile);
  request.instructions = InstructionPoly(program);

  // Add profiles to planning server
  auto default_simple_plan_profile = std::make_shared<SimplePlannerFixedSizeAssignPlanProfile>();
  ProfileDictionary::Ptr profiles = planning_server.getProfiles();
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, freespace_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, approach_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, process_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, departure_profile, default_simple_plan_profile);

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Confirm that the task is finished
  EXPECT_TRUE(response.ready());

  // Solve
  EXPECT_TRUE(response.interface->isSuccessful());
}

TEST_F(TesseractProcessManagerUnit, RasterWAADProcessManagerDefaultLVSPlanProfileTest)  // NOLINT
{
  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env_), 1);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = process_planner_names::RASTER_FT_WAAD_PLANNER_NAME;

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string approach_profile = "APPROACH";
  std::string process_profile = "PROCESS";
  std::string departure_profile = "DEPARTURE";

  CompositeInstruction program =
      rasterWAADExampleProgram(freespace_profile, approach_profile, process_profile, departure_profile);
  request.instructions = InstructionPoly(program);

  // Add profiles to planning server
  auto default_simple_plan_profile = std::make_shared<SimplePlannerLVSPlanProfile>();
  ProfileDictionary::Ptr profiles = planning_server.getProfiles();
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, freespace_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, approach_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, process_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, departure_profile, default_simple_plan_profile);

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Confirm that the task is finished
  EXPECT_TRUE(response.ready());

  // Solve
  EXPECT_TRUE(response.interface->isSuccessful());
}

TEST_F(TesseractProcessManagerUnit, RasterWAADDTProcessManagerDefaultPlanProfileTest)  // NOLINT
{
  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env_), 1);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = process_planner_names::RASTER_FT_WAAD_DT_PLANNER_NAME;

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string approach_profile = "APPROACH";
  std::string process_profile = "PROCESS";
  std::string departure_profile = "DEPARTURE";

  CompositeInstruction program =
      rasterWAADDTExampleProgram(freespace_profile, approach_profile, process_profile, departure_profile);
  request.instructions = InstructionPoly(program);

  // Add profiles to planning server
  auto default_simple_plan_profile = std::make_shared<SimplePlannerFixedSizeAssignPlanProfile>();
  ProfileDictionary::Ptr profiles = planning_server.getProfiles();
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, freespace_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, approach_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, process_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, departure_profile, default_simple_plan_profile);

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Confirm that the task is finished
  EXPECT_TRUE(response.ready());

  // Solve
  EXPECT_TRUE(response.interface->isSuccessful());
}

TEST_F(TesseractProcessManagerUnit, RasterWAADDTProcessManagerDefaultLVSPlanProfileTest)  // NOLINT
{
  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env_), 1);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = process_planner_names::RASTER_FT_WAAD_DT_PLANNER_NAME;

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string approach_profile = "APPROACH";
  std::string process_profile = "PROCESS";
  std::string departure_profile = "DEPARTURE";

  CompositeInstruction program =
      rasterWAADDTExampleProgram(freespace_profile, approach_profile, process_profile, departure_profile);
  request.instructions = InstructionPoly(program);

  // Add profiles to planning server
  auto default_simple_plan_profile = std::make_shared<SimplePlannerLVSPlanProfile>();
  ProfileDictionary::Ptr profiles = planning_server.getProfiles();
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, freespace_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, approach_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, process_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(
      SIMPLE_DEFAULT_NAMESPACE, departure_profile, default_simple_plan_profile);

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Confirm that the task is finished
  EXPECT_TRUE(response.ready());

  // Solve
  EXPECT_TRUE(response.interface->isSuccessful());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
