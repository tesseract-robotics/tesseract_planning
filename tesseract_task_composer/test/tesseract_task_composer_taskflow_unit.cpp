#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <sstream>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/taskflow/taskflow_task_composer_executor.h>
#include <tesseract_task_composer/taskflow/taskflow_task_composer_future.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>
#include <tesseract_task_composer/core/nodes/done_task.h>
#include <tesseract_task_composer/core/test_suite/task_composer_serialization_utils.hpp>

using namespace tesseract_planning;

TEST(TesseractTaskComposerTaskflowUnit, TaskComposerExecutorTests)  // NOLINT
{
  {  // task
    auto task = std::make_unique<DoneTask>("DoneTask");
    auto executor = std::make_unique<TaskflowTaskComposerExecutor>("TaskComposerExecutorTests", 3);
    EXPECT_EQ(executor->getName(), "TaskComposerExecutorTests");
    EXPECT_EQ(executor->getWorkerCount(), 3);
    EXPECT_EQ(executor->getTaskCount(), 0);

    auto input = std::make_unique<TaskComposerInput>(std::make_unique<TaskComposerProblem>());
    auto future = executor->run(*task, *input);
    future->wait();
    EXPECT_TRUE(future->valid());
    EXPECT_TRUE(future->ready());
    std::future_status sd = future->waitFor(std::chrono::duration<double>(1));
    EXPECT_EQ(sd, std::future_status::ready);
    std::future_status sc = future->waitUntil(std::chrono::high_resolution_clock::now());
    EXPECT_EQ(sc, std::future_status::ready);
    auto copy_future = future->copy();
    EXPECT_TRUE(copy_future->valid());
    EXPECT_TRUE(copy_future->ready());

    EXPECT_FALSE(task->isConditional());
    EXPECT_EQ(input->isAborted(), false);
    EXPECT_EQ(input->isSuccessful(), true);
    EXPECT_EQ(input->task_infos.getInfoMap().size(), 1);
    EXPECT_TRUE(input->task_infos.getAbortingNode().is_nil());

    future->clear();
    EXPECT_FALSE(future->valid());

    // Serialization
    test_suite::runSerializationPointerTest(executor, "TaskComposerExecutorTests");
  }

  {  // Pipeline
    std::string str = R"(task_composer_plugins:
                           search_paths:
                             - /usr/local/lib
                           search_libraries:
                             - tesseract_task_composer_factories
                           tasks:
                             plugins:
                               TestPipeline:
                                 class: PipelineTaskFactory
                                 config:
                                   conditional: true
                                   inputs: input_data
                                   outputs: output_data
                                   nodes:
                                     StartTask:
                                       class: StartTaskFactory
                                       config:
                                         conditional: false
                                     DoneTask:
                                       class: DoneTaskFactory
                                       config:
                                         conditional: false
                                   edges:
                                     - source: StartTask
                                       destinations: [DoneTask]
                                   terminals: [DoneTask])";

    TaskComposerPluginFactory factory(str);

    std::string str2 = R"(config:
                            conditional: true
                            inputs: input_data
                            outputs: output_data
                            nodes:
                              StartTask:
                                task:
                                  name: TestPipeline
                                  conditional: false
                                  input_remapping:
                                    input_data: output_data
                                  output_remapping:
                                    output_data: input_data
                              DoneTask:
                                class: DoneTaskFactory
                                config:
                                  conditional: false
                            edges:
                              - source: StartTask
                                destinations: [DoneTask]
                            terminals: [DoneTask])";
    YAML::Node config = YAML::Load(str2);
    auto pipeline = std::make_unique<TaskComposerPipeline>("Pipeline", config["config"], factory);
    auto executor = std::make_unique<TaskflowTaskComposerExecutor>("TaskComposerExecutorTests", 3);
    EXPECT_EQ(executor->getName(), "TaskComposerExecutorTests");
    EXPECT_EQ(executor->getWorkerCount(), 3);
    EXPECT_EQ(executor->getTaskCount(), 0);

    auto input = std::make_unique<TaskComposerInput>(std::make_unique<TaskComposerProblem>());
    auto future = executor->run(*pipeline, *input);
    future->wait();
    EXPECT_TRUE(future->valid());
    EXPECT_TRUE(future->ready());
    std::future_status sd = future->waitFor(std::chrono::duration<double>(1));
    EXPECT_EQ(sd, std::future_status::ready);
    std::future_status sc = future->waitUntil(std::chrono::high_resolution_clock::now());
    EXPECT_EQ(sc, std::future_status::ready);
    auto copy_future = future->copy();
    EXPECT_TRUE(copy_future->valid());
    EXPECT_TRUE(copy_future->ready());

    EXPECT_TRUE(pipeline->isConditional());
    EXPECT_EQ(pipeline->getTerminals().size(), 1);
    auto task1 = pipeline->getNodeByName("StartTask");
    auto task2 = pipeline->getNodeByName("DoneTask");
    EXPECT_EQ(pipeline->getTerminals(), std::vector<boost::uuids::uuid>({ task2->getUUID() }));
    EXPECT_EQ(task1->getInboundEdges().size(), 0);
    EXPECT_EQ(task1->getOutboundEdges().size(), 1);
    EXPECT_EQ(task1->getOutboundEdges().front(), task2->getUUID());
    EXPECT_EQ(task2->getInboundEdges().size(), 1);
    EXPECT_EQ(task2->getInboundEdges().front(), task1->getUUID());
    EXPECT_EQ(task2->getOutboundEdges().size(), 0);
    EXPECT_EQ(input->isAborted(), false);
    EXPECT_EQ(input->isSuccessful(), true);
    EXPECT_EQ(input->task_infos.getInfoMap().size(), 5);
    EXPECT_TRUE(input->task_infos.getAbortingNode().is_nil());

    future->clear();
    EXPECT_FALSE(future->valid());

    // Serialization
    test_suite::runSerializationPointerTest(executor, "TaskComposerExecutorTests");
  }

  {  // Graph
    std::string str = R"(task_composer_plugins:
                           search_paths:
                             - /usr/local/lib
                           search_libraries:
                             - tesseract_task_composer_factories
                           tasks:
                             plugins:
                               TestPipeline:
                                 class: PipelineTaskFactory
                                 config:
                                   conditional: true
                                   inputs: input_data
                                   outputs: output_data
                                   nodes:
                                     StartTask:
                                       class: StartTaskFactory
                                       config:
                                         conditional: false
                                     DoneTask:
                                       class: DoneTaskFactory
                                       config:
                                         conditional: false
                                   edges:
                                     - source: StartTask
                                       destinations: [DoneTask]
                                   terminals: [DoneTask])";

    TaskComposerPluginFactory factory(str);

    std::string str2 = R"(config:
                            conditional: false
                            inputs: input_data
                            outputs: output_data
                            nodes:
                              StartTask:
                                task:
                                  name: TestPipeline
                                  conditional: false
                                  input_remapping:
                                    input_data: output_data
                                  output_remapping:
                                    output_data: input_data
                              DoneTask:
                                class: DoneTaskFactory
                                config:
                                  conditional: false
                            edges:
                              - source: StartTask
                                destinations: [DoneTask]
                            terminals: [DoneTask])";
    YAML::Node config = YAML::Load(str2);
    auto graph = std::make_unique<TaskComposerGraph>("Graph", config["config"], factory);
    auto executor = std::make_unique<TaskflowTaskComposerExecutor>("TaskComposerExecutorTests", 3);
    EXPECT_EQ(executor->getName(), "TaskComposerExecutorTests");
    EXPECT_EQ(executor->getWorkerCount(), 3);
    EXPECT_EQ(executor->getTaskCount(), 0);

    auto input = std::make_unique<TaskComposerInput>(std::make_unique<TaskComposerProblem>());
    auto future = executor->run(*graph, *input);
    future->wait();
    EXPECT_TRUE(future->valid());
    EXPECT_TRUE(future->ready());
    std::future_status sd = future->waitFor(std::chrono::duration<double>(1));
    EXPECT_EQ(sd, std::future_status::ready);
    std::future_status sc = future->waitUntil(std::chrono::high_resolution_clock::now());
    EXPECT_EQ(sc, std::future_status::ready);
    auto copy_future = future->copy();
    EXPECT_TRUE(copy_future->valid());
    EXPECT_TRUE(copy_future->ready());

    EXPECT_FALSE(graph->isConditional());
    EXPECT_EQ(graph->getTerminals().size(), 1);
    auto task1 = graph->getNodeByName("StartTask");
    auto task2 = graph->getNodeByName("DoneTask");
    EXPECT_EQ(graph->getTerminals(), std::vector<boost::uuids::uuid>({ task2->getUUID() }));
    EXPECT_EQ(task1->getInboundEdges().size(), 0);
    EXPECT_EQ(task1->getOutboundEdges().size(), 1);
    EXPECT_EQ(task1->getOutboundEdges().front(), task2->getUUID());
    EXPECT_EQ(task2->getInboundEdges().size(), 1);
    EXPECT_EQ(task2->getInboundEdges().front(), task1->getUUID());
    EXPECT_EQ(task2->getOutboundEdges().size(), 0);
    EXPECT_EQ(input->isAborted(), false);
    EXPECT_EQ(input->isSuccessful(), true);
    EXPECT_EQ(input->task_infos.getInfoMap().size(), 4);  // Why is this one less than the pipeline
    EXPECT_TRUE(input->task_infos.getAbortingNode().is_nil());

    future->clear();
    EXPECT_FALSE(future->valid());

    // Serialization
    test_suite::runSerializationPointerTest(executor, "TaskComposerExecutorTests");
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
