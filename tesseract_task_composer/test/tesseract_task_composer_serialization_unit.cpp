/**
 * @file teasseract_task_composer_serialize_test.cpp
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
#include <boost/uuid/uuid_generators.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/unit_test_utils.h>
#include <tesseract_common/serialization.h>
#include <tesseract_command_language/profile_dictionary.h>
#include <tesseract_task_composer/task_composer_node_info.h>
#include <tesseract_task_composer/task_composer_input.h>
#include <tesseract_task_composer/task_composer_data_storage.h>
#include <tesseract_task_composer/nodes/continuous_contact_check_task.h>
#include <tesseract_task_composer/nodes/discrete_contact_check_task.h>
#include <tesseract_task_composer/nodes/fix_state_collision_task.h>
#include <tesseract_task_composer/taskflow/taskflow_task_composer_future.h>

#include "raster_example_program.h"
#include "freespace_example_program.h"

using namespace tesseract_planning;

void setNodeInfoData(TaskComposerNodeInfo& info)
{
  info.elapsed_time = 123.456;
  info.message = "Test message";
  info.return_value = 1;
  info.results = rasterExampleProgram();
}

TEST(TesseractTaskComposerSerializeUnit, TaskComposerNodeInfo)  // NOLINT
{
  boost::uuids::uuid uuid = boost::uuids::random_generator()();
  auto info = std::make_shared<TaskComposerNodeInfo>(uuid, "my task");
  setNodeInfoData(*info);
  tesseract_common::testSerialization<TaskComposerNodeInfo>(*info, "TaskComposerNodeInfo");
}

TEST(TesseractTaskComposerSerializeUnit, ContinuousContactCheckTaskInfo)  // NOLINT
{
  boost::uuids::uuid uuid = boost::uuids::random_generator()();
  auto info = std::make_shared<ContinuousContactCheckTaskInfo>(uuid, "ContinuousContactCheckTaskInfo");
  setNodeInfoData(*info);
  tesseract_common::testSerialization<ContinuousContactCheckTaskInfo>(*info, "ContinuousContactCheckTaskInfo");
  tesseract_common::testSerializationDerivedClass<TaskComposerNodeInfo, ContinuousContactCheckTaskInfo>(info,
                                                                                                        "ContinuousCont"
                                                                                                        "actCheckTask"
                                                                                                        "Info");
}

TEST(TesseractTaskComposerSerializeUnit, DiscreteContactCheckTaskInfo)  // NOLINT
{
  boost::uuids::uuid uuid = boost::uuids::random_generator()();
  auto info = std::make_shared<DiscreteContactCheckTaskInfo>(uuid, "DiscreteContactCheckTaskInfo");
  setNodeInfoData(*info);
  tesseract_common::testSerialization<DiscreteContactCheckTaskInfo>(*info, "DiscreteContactCheckTaskInfo");
  tesseract_common::testSerializationDerivedClass<TaskComposerNodeInfo, DiscreteContactCheckTaskInfo>(info,
                                                                                                      "DiscreteContactC"
                                                                                                      "heckTaskInfo");
}

TEST(TesseractTaskComposerSerializeUnit, FixStateCollisionTaskInfo)  // NOLINT
{
  boost::uuids::uuid uuid = boost::uuids::random_generator()();
  auto info = std::make_shared<FixStateCollisionTaskInfo>(uuid, "FixStateCollisionTaskInfo");
  setNodeInfoData(*info);
  tesseract_common::testSerialization<FixStateCollisionTaskInfo>(*info, "FixStateCollisionTaskInfo");
  tesseract_common::testSerializationDerivedClass<TaskComposerNodeInfo, FixStateCollisionTaskInfo>(info,
                                                                                                   "FixStateCollisionTa"
                                                                                                   "skInfo");
}

TEST(TesseractTaskComposerSerializeUnit, TaskComposerInput)  // NOLINT
{
  boost::uuids::uuid uuid = boost::uuids::random_generator()();
  auto info = std::make_unique<TaskComposerNodeInfo>(uuid, "my task");
  setNodeInfoData(*info);

  // Define profiles
  auto profiles = std::make_shared<ProfileDictionary>();

  // Create data storage
  TaskComposerDataStorage task_data;
  task_data.setData("input_program", rasterExampleProgram());

  // Create problem
  TaskComposerProblem task_problem(nullptr, task_data);

  // Create task input
  auto task_input = std::make_shared<TaskComposerInput>(task_problem, profiles);

  task_input->task_infos.addInfo(std::move(info));
  task_input->abort();

  tesseract_common::testSerialization<TaskComposerInput>(*task_input, "TaskComposerInput");
}

TEST(TesseractTaskComposerSerializeUnit, TaskComposerFuture)  // NOLINT
{
  TaskflowTaskComposerFuture future;
  tesseract_common::testSerialization<TaskflowTaskComposerFuture>(future, "TaskflowTaskComposerFuture");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
