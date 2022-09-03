/**
 * @file serialize_test.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date June 22, 2021
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

#include <tesseract_common/unit_test_utils.h>
#include <tesseract_common/serialization.h>
#include <tesseract_environment/commands/add_allowed_collision_command.h>
#include <tesseract_process_managers/core/task_info.h>
#include <tesseract_process_managers/core/process_planning_request.h>
#include <tesseract_process_managers/core/process_planning_future.h>
#include <tesseract_process_managers/task_generators/check_input_task_generator.h>
#include <tesseract_process_managers/task_generators/continuous_contact_check_task_generator.h>
#include <tesseract_process_managers/task_generators/discrete_contact_check_task_generator.h>
#include <tesseract_process_managers/task_generators/fix_state_bounds_task_generator.h>
#include <tesseract_process_managers/task_generators/fix_state_collision_task_generator.h>
#include <tesseract_process_managers/task_generators/has_seed_task_generator.h>
#include <tesseract_process_managers/task_generators/iterative_spline_parameterization_task_generator.h>
#include <tesseract_process_managers/task_generators/motion_planner_task_generator.h>
#include <tesseract_process_managers/task_generators/profile_switch_task_generator.h>
#include <tesseract_process_managers/task_generators/seed_min_length_task_generator.h>
#include <tesseract_process_managers/task_generators/time_optimal_parameterization_task_generator.h>
#include <tesseract_process_managers/task_generators/upsample_trajectory_task_generator.h>

#include "raster_example_program.h"
#include "raster_dt_example_program.h"
#include "freespace_example_program.h"

using namespace tesseract_planning;

void setTaskInfoData(const TaskInfo::Ptr& task_info)
{
  task_info->elapsed_time = 123.456;
  task_info->message = "Test message";
  task_info->instructions_input = rasterExampleProgram();
  task_info->instructions_output = freespaceExampleProgramABB();
  task_info->results_input = freespaceExampleProgramIIWA();
  task_info->results_output = rasterDTExampleProgram();
}

TEST(TesseractProcessManagersSerializeUnit, TaskInfo)  // NOLINT
{
  auto task_info = std::make_shared<TaskInfo>(123456, "my task");
  tesseract_common::testSerialization<TaskInfo>(*task_info, "TaskInfo");
}

TEST(TesseractProcessManagersSerializeUnit, CheckInputTaskInfo)  // NOLINT
{
  auto task_info = std::make_shared<CheckInputTaskInfo>(123456, "CheckInputTaskInfo");
  setTaskInfoData(task_info);
  tesseract_common::testSerialization<CheckInputTaskInfo>(*task_info, "CheckInputTaskInfo");
  tesseract_common::testSerializationDerivedClass<TaskInfo, CheckInputTaskInfo>(task_info, "CheckInputTaskInfo");
}

TEST(TesseractProcessManagersSerializeUnit, ContinuousContactCheckTaskInfo)  // NOLINT
{
  auto task_info = std::make_shared<ContinuousContactCheckTaskInfo>(123456, "ContinuousContactCheckTaskInfo");
  setTaskInfoData(task_info);
  tesseract_common::testSerialization<ContinuousContactCheckTaskInfo>(*task_info, "ContinuousContactCheckTaskInfo");
  tesseract_common::testSerializationDerivedClass<TaskInfo, ContinuousContactCheckTaskInfo>(task_info,
                                                                                            "ContinuousContactCheckTask"
                                                                                            "Info");
}

TEST(TesseractProcessManagersSerializeUnit, FixStateBoundsTaskInfo)  // NOLINT
{
  auto task_info = std::make_shared<FixStateBoundsTaskInfo>(123456, "FixStateBoundsTaskInfo");
  setTaskInfoData(task_info);
  tesseract_common::testSerialization<FixStateBoundsTaskInfo>(*task_info, "FixStateBoundsTaskInfo");
  tesseract_common::testSerializationDerivedClass<TaskInfo, FixStateBoundsTaskInfo>(task_info,
                                                                                    "FixStateBoundsTaskInfo");
}

TEST(TesseractProcessManagersSerializeUnit, FixStateCollisionTaskInfo)  // NOLINT
{
  auto task_info = std::make_shared<FixStateCollisionTaskInfo>(123456, "FixStateCollisionTaskInfo");
  setTaskInfoData(task_info);
  tesseract_common::testSerialization<FixStateCollisionTaskInfo>(*task_info, "FixStateCollisionTaskInfo");
  tesseract_common::testSerializationDerivedClass<TaskInfo, FixStateCollisionTaskInfo>(task_info,
                                                                                       "FixStateCollisionTaskInfo");
}

TEST(TesseractProcessManagersSerializeUnit, HasSeedTaskInfo)  // NOLINT
{
  auto task_info = std::make_shared<HasSeedTaskInfo>(123456, "HasSeedTaskInfo");
  setTaskInfoData(task_info);
  tesseract_common::testSerialization<HasSeedTaskInfo>(*task_info, "HasSeedTaskInfo");
  tesseract_common::testSerializationDerivedClass<TaskInfo, HasSeedTaskInfo>(task_info, "HasSeedTaskInfo");
}

TEST(TesseractProcessManagersSerializeUnit, IterativeSplineParameterizationTaskInfo)  // NOLINT
{
  auto task_info =
      std::make_shared<IterativeSplineParameterizationTaskInfo>(123456, "IterativeSplineParameterizationTaskInfo");
  setTaskInfoData(task_info);
  tesseract_common::testSerialization<IterativeSplineParameterizationTaskInfo>(*task_info,
                                                                               "IterativeSplineParameterizationTaskInf"
                                                                               "o");
  tesseract_common::testSerializationDerivedClass<TaskInfo, IterativeSplineParameterizationTaskInfo>(task_info,
                                                                                                     "IterativeSplinePa"
                                                                                                     "rameterizationTas"
                                                                                                     "kInfo");
}

TEST(TesseractProcessManagersSerializeUnit, MotionPlannerTaskInfo)  // NOLINT
{
  auto task_info = std::make_shared<MotionPlannerTaskInfo>(123456, "MotionPlannerTaskInfo");
  setTaskInfoData(task_info);
  tesseract_common::testSerialization<MotionPlannerTaskInfo>(*task_info, "MotionPlannerTaskInfo");
  tesseract_common::testSerializationDerivedClass<TaskInfo, MotionPlannerTaskInfo>(task_info, "MotionPlannerTaskInfo");
}

TEST(TesseractProcessManagersSerializeUnit, ProfileSwitchTaskInfo)  // NOLINT
{
  auto task_info = std::make_shared<ProfileSwitchTaskInfo>(123456, "ProfileSwitchTaskInfo");
  setTaskInfoData(task_info);
  tesseract_common::testSerialization<ProfileSwitchTaskInfo>(*task_info, "ProfileSwitchTaskInfo");
  tesseract_common::testSerializationDerivedClass<TaskInfo, ProfileSwitchTaskInfo>(task_info, "ProfileSwitchTaskInfo");
}

TEST(TesseractProcessManagersSerializeUnit, SeedMinLengthTaskInfo)  // NOLINT
{
  auto task_info = std::make_shared<SeedMinLengthTaskInfo>(123456, "SeedMinLengthTaskInfo");
  setTaskInfoData(task_info);
  tesseract_common::testSerialization<SeedMinLengthTaskInfo>(*task_info, "SeedMinLengthTaskInfo");
  tesseract_common::testSerializationDerivedClass<TaskInfo, SeedMinLengthTaskInfo>(task_info, "SeedMinLengthTaskInfo");
}

TEST(TesseractProcessManagersSerializeUnit, TimeOptimalTrajectoryGenerationTaskInfo)  // NOLINT
{
  auto task_info =
      std::make_shared<TimeOptimalTrajectoryGenerationTaskInfo>(123456, "TimeOptimalTrajectoryGenerationTaskInfo");
  setTaskInfoData(task_info);
  tesseract_common::testSerialization<TimeOptimalTrajectoryGenerationTaskInfo>(*task_info,
                                                                               "TimeOptimalTrajectoryGenerationTaskInf"
                                                                               "o");
  tesseract_common::testSerializationDerivedClass<TaskInfo, TimeOptimalTrajectoryGenerationTaskInfo>(task_info,
                                                                                                     "TimeOptimalTrajec"
                                                                                                     "toryGenerationTas"
                                                                                                     "kInfo");
}

TEST(TesseractProcessManagersSerializeUnit, UpsampleTrajectoryTaskInfo)  // NOLINT
{
  auto task_info = std::make_shared<UpsampleTrajectoryTaskInfo>(123456, "UpsampleTrajectoryTaskInfo");
  setTaskInfoData(task_info);
  tesseract_common::testSerialization<UpsampleTrajectoryTaskInfo>(*task_info, "UpsampleTrajectoryTaskInfo");
  tesseract_common::testSerializationDerivedClass<TaskInfo, UpsampleTrajectoryTaskInfo>(task_info,
                                                                                        "UpsampleTrajectoryTaskInfo");
}

TEST(TesseractProcessManagersSerializeUnit, TaskflowInterface)  // NOLINT
{
  auto task_info = std::make_unique<CheckInputTaskInfo>(123456, "my task");
  task_info->elapsed_time = 123.456;
  task_info->message = "Test message";
  task_info->instructions_input = rasterExampleProgram();
  task_info->instructions_output = freespaceExampleProgramABB();
  task_info->results_input = freespaceExampleProgramIIWA();
  task_info->results_output = rasterDTExampleProgram();

  TaskflowInterface interface;
  interface.getTaskInfoContainer()->addTaskInfo(std::move(task_info));
  interface.abort();

  tesseract_common::testSerialization<TaskflowInterface>(interface, "TaskflowInterface");
}

TEST(TesseractProcessManagersSerializeUnit, ProcessPlanningRequest)  // NOLINT
{
  // Create the test case
  ProcessPlanningRequest request;
  request.name = "process_planning_request";
  request.instructions = rasterExampleProgram();
  request.seed = freespaceExampleProgramIIWA();
  request.env_state.joints["joint 1"] = 5;
  request.commands.push_back(
      std::make_shared<tesseract_environment::AddAllowedCollisionCommand>("link1", "link2", "reason"));
  request.profile = true;
  request.save_io = true;
  std::unordered_map<std::string, std::string> remapping;
  remapping["test1a_key"] = "test1a_value";
  request.plan_profile_remapping["test1_key"] = remapping;
  remapping["test2a_key"] = "test2a_value";
  request.plan_profile_remapping["test2_key"] = remapping;

  tesseract_common::testSerialization<ProcessPlanningRequest>(request, "ProcessPlanningRequest");
}

TEST(TesseractProcessManagersSerializeUnit, ProcessPlanningFuture)  // NOLINT
{
  // Create the test case
  ProcessPlanningFuture request;
  request.problem->input = std::make_unique<InstructionPoly>(rasterExampleProgram());
  request.problem->results = std::make_unique<InstructionPoly>(freespaceExampleProgramIIWA());

  tesseract_common::testSerialization<ProcessPlanningFuture>(request, "ProcessPlanningFuture");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
