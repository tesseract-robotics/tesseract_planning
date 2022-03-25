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
#include <tesseract_command_language/core/serialization.h>
#include <tesseract_environment/commands/add_allowed_collision_command.h>
#include <tesseract_process_managers/core/task_info.h>
#include <tesseract_process_managers/core/process_planning_request.h>
#include <tesseract_process_managers/core/process_planning_future.h>

#include "raster_example_program.h"
#include "raster_dt_example_program.h"
#include "freespace_example_program.h"

using namespace tesseract_planning;

TEST(TesseractProcessManagersSerializeUnit, TaskInfo)  // NOLINT
{
  TaskInfo task_info(123456, "my task");
  task_info.elapsed_time = 123.456;
  task_info.message = "Test message";
  task_info.instructions_input = rasterExampleProgram();
  task_info.instructions_output = freespaceExampleProgramABB();
  task_info.results_input = freespaceExampleProgramIIWA();
  task_info.results_output = rasterDTExampleProgram();

  tesseract_common::testSerialization<TaskInfo>(task_info, "TaskInfo");
}

TEST(TesseractProcessManagersSerializeUnit, TaskflowInterface)  // NOLINT
{
  auto task_info = std::make_unique<TaskInfo>(123456, "my task");
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
  request.input = std::make_unique<Instruction>(rasterExampleProgram());
  request.results = std::make_unique<Instruction>(freespaceExampleProgramIIWA());

  tesseract_common::testSerialization<ProcessPlanningFuture>(request, "ProcessPlanningFuture");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
