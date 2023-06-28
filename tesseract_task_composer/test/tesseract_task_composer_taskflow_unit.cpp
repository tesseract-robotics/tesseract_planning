#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <sstream>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/taskflow/taskflow_task_composer_executor.h>
#include <tesseract_task_composer/core/test_suite/task_composer_executor_unit.hpp>

using namespace tesseract_planning;

TEST(TesseractTaskComposerTaskflowUnit, TaskComposerExecutorTests)  // NOLINT
{
  test_suite::runTaskComposerExecutorTest<TaskflowTaskComposerExecutor>();

  // Test YAML Config loading
  {
    std::string str = R"(config:
                           threads: 3)";
    YAML::Node config = YAML::Load(str);
    TaskflowTaskComposerExecutor executor("TaskComposerExecutorTests", config["config"]);
    EXPECT_EQ(executor.getName(), "TaskComposerExecutorTests");
    EXPECT_EQ(executor.getWorkerCount(), 3);
  }

  {  // Failure
    std::string str = R"(config:
                           threads: -3)";
    YAML::Node config = YAML::Load(str);
    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(std::make_unique<TaskflowTaskComposerExecutor>("TaskComposerExecutorTests", config["config"]));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
