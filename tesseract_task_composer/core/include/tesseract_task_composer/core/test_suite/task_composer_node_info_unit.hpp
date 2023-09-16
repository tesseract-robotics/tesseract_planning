/**
 * @file task_composer_node_info_unit.hpp
 * @brief Collection of unit tests for TaskComposerNodeInfo
 *
 * @author Levi Armstrong
 * @date June 12, 2023
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2023, Levi Armstrong
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
#ifndef TESSERACT_TASK_COMPOSER_TASK_COMPOSER_NODE_INFO_UNIT_HPP
#define TESSERACT_TASK_COMPOSER_TASK_COMPOSER_NODE_INFO_UNIT_HPP

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_node.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>
#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/test_suite/task_composer_serialization_utils.hpp>

namespace tesseract_planning::test_suite
{
template <typename T>
void runTaskComposerNodeInfoTest()
{
  {  // Default
    T node_info;
    EXPECT_EQ(node_info.return_value, -1);
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(node_info.elapsed_time, 0));
    EXPECT_TRUE(node_info.uuid.is_nil());
    EXPECT_TRUE(node_info.parent_uuid.is_nil());
    EXPECT_EQ(node_info.color, "red");
    EXPECT_FALSE(node_info.isAborted());
    EXPECT_EQ(node_info, *(node_info.clone()));

    // Serialization
    test_suite::runSerializationTest<T>(node_info, "TaskComposerNodeInfoTests");
  }

  {  // Constructor
    TaskComposerNode node;
    T node_info(node);
    EXPECT_EQ(node_info.return_value, -1);
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(node_info.elapsed_time, 0));
    EXPECT_FALSE(node_info.uuid.is_nil());
    EXPECT_EQ(node_info.uuid, node.getUUID());
    EXPECT_EQ(node_info.parent_uuid, node.getParentUUID());
    EXPECT_EQ(node_info.color, "red");
    EXPECT_FALSE(node_info.isAborted());
    EXPECT_EQ(node_info, *(node_info.clone()));

    // Serialization
    test_suite::runSerializationTest<T>(node_info, "TaskComposerNodeInfoTests");
  }
}
}  // namespace tesseract_planning::test_suite
#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_NODE_INFO_UNIT_HPP
