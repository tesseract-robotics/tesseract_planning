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

#include <tesseract_command_language/core/serialization.h>
#include <tesseract_process_managers/core/task_info.h>

#include "raster_example_program.h"
#include "raster_dt_example_program.h"
#include "freespace_example_program.h"

using namespace tesseract_planning;

TEST(TesseractProcessManagersSerializeUnit, serializationCompositeInstruction)  // NOLINT
{
  TaskInfo task_info(123456, "my task");
  task_info.elapsed_time = 123.456;
  task_info.message = "Test message";
  task_info.instructions_input = rasterExampleProgram();
  task_info.instructions_output = freespaceExampleProgramABB();
  task_info.results_input = freespaceExampleProgramIIWA();
  task_info.results_output = rasterDTExampleProgram();

  {  // Archive program to file
    std::string file_path = tesseract_common::getTempPath() + "task_info_unit.xml";
    EXPECT_TRUE(Serialization::toArchiveFileXML<TaskInfo>(task_info, file_path));

    auto ntask_info = Serialization::fromArchiveFileXML<TaskInfo>(file_path);
    EXPECT_TRUE(task_info == ntask_info);
  }

  {  // Archive program to string
    std::string task_info_string = Serialization::toArchiveStringXML<TaskInfo>(task_info, "task_info");
    EXPECT_FALSE(task_info_string.empty());

    auto ntask_info = Serialization::fromArchiveStringXML<TaskInfo>(task_info_string);
    EXPECT_TRUE(task_info == ntask_info);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
