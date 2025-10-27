/**
 * @file task_composer_executor_unit.hpp
 * @brief Collection of unit tests for TaskComposerExecutor
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
#ifndef TESSERACT_TASK_COMPOSER_TASK_COMPOSER_EXECUTOR_UNIT_HPP
#define TESSERACT_TASK_COMPOSER_TASK_COMPOSER_EXECUTOR_UNIT_HPP

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/test_suite/task_composer_serialization_utils.hpp>
#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_future.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_pipeline.h>
#include <tesseract_task_composer/core/task_composer_graph.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>
#include <tesseract_task_composer/core/nodes/done_task.h>

#include <tesseract_common/resource_locator.h>
#include <tesseract_common/yaml_utils.h>

namespace tesseract_planning::test_suite
{
template <typename T>
void runTaskComposerExecutorTest()
{
  {  // task
    auto task = std::make_unique<DoneTask>("DoneTask");
    tesseract_planning::TaskComposerExecutor::UPtr executor = std::make_unique<T>("TaskComposerExecutorTests", 3);
    EXPECT_EQ(executor->getName(), "TaskComposerExecutorTests");
    EXPECT_EQ(executor->getWorkerCount(), 3);
    EXPECT_EQ(executor->getTaskCount(), 0);

    auto future = executor->run(*task, std::make_shared<TaskComposerContext>(task->getName()));
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
    EXPECT_EQ(future->context->isAborted(), false);
    EXPECT_EQ(future->context->isSuccessful(), true);
    EXPECT_EQ(future->context->task_infos->getInfoMap().size(), 1);
    EXPECT_TRUE(future->context->task_infos->getAbortingNode().is_nil());

    future->clear();
    EXPECT_FALSE(future->valid());

    // Serialization
    test_suite::runSerializationPointerTest(executor, "TaskComposerExecutorTests");
  }

  tesseract_common::GeneralResourceLocator locator;
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
                                 nodes:
                                   StartTask:
                                     class: StartTaskFactory
                                     config:
                                       conditional: false
                                   TestTask:
                                     class: TestTaskFactory
                                     config:
                                       conditional: true
                                       return_value: 1
                                       inputs:
                                         port1: input_data
                                         port2: [input_data]
                                       outputs:
                                         port1: output_data
                                         port2: [output_data]
                                   DoneTask:
                                     class: DoneTaskFactory
                                     config:
                                       conditional: false
                                   AbortTask:
                                     class: DoneTaskFactory
                                     config:
                                       conditional: false
                                 edges:
                                   - source: StartTask
                                     destinations: [TestTask]
                                   - source: TestTask
                                     destinations: [AbortTask, DoneTask]
                                 terminals: [AbortTask, DoneTask]
                             TestGraph:
                               class: GraphTaskFactory
                               config:
                                 conditional: false
                                 nodes:
                                   StartTask:
                                     class: StartTaskFactory
                                     config:
                                       conditional: false
                                   TestTask:
                                     class: TestTaskFactory
                                     config:
                                       conditional: true
                                       return_value: 1
                                       inputs:
                                         port1: input_data
                                         port2: [input_data]
                                       outputs:
                                         port1: output_data
                                         port2: [output_data]
                                   DoneTask:
                                     class: DoneTaskFactory
                                     config:
                                       conditional: false
                                   AbortTask:
                                     class: DoneTaskFactory
                                     config:
                                       conditional: false
                                 edges:
                                   - source: StartTask
                                     destinations: [TestTask]
                                   - source: TestTask
                                     destinations: [AbortTask, DoneTask]
                                 terminals: [AbortTask, DoneTask])";

  TaskComposerPluginFactory factory(str, locator);

  {  // Pipeline
    std::string str2 = R"(config:
                            conditional: true
                            nodes:
                              StartTask:
                                task: TestPipeline
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
    YAML::Node config = YAML::Load(str2);
    auto pipeline = std::make_unique<TaskComposerPipeline>("Pipeline", config["config"], factory);
    tesseract_planning::TaskComposerExecutor::UPtr executor = std::make_unique<T>("TaskComposerExecutorTests", 3);
    EXPECT_EQ(executor->getName(), "TaskComposerExecutorTests");
    EXPECT_EQ(executor->getWorkerCount(), 3);
    EXPECT_EQ(executor->getTaskCount(), 0);

    auto future = executor->run(*pipeline, std::make_shared<TaskComposerContext>(pipeline->getName()));
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
    EXPECT_EQ(future->context->isAborted(), false);
    EXPECT_EQ(future->context->isSuccessful(), true);
    EXPECT_EQ(future->context->task_infos->getInfoMap().size(), 6);
    EXPECT_TRUE(future->context->task_infos->getAbortingNode().is_nil());

    future->clear();
    EXPECT_FALSE(future->valid());

    // Serialization
    test_suite::runSerializationPointerTest(executor, "TaskComposerExecutorTests");
  }

  {  // Graph with child pipeline task not conditional
    std::string str2 = R"(config:
                            conditional: false
                            nodes:
                              StartTask:
                                task: TestPipeline
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
    YAML::Node config = YAML::Load(str2);
    auto graph = std::make_unique<TaskComposerGraph>("Graph", config["config"], factory);
    tesseract_planning::TaskComposerExecutor::UPtr executor = std::make_unique<T>("TaskComposerExecutorTests", 3);
    EXPECT_EQ(executor->getName(), "TaskComposerExecutorTests");
    EXPECT_EQ(executor->getWorkerCount(), 3);
    EXPECT_EQ(executor->getTaskCount(), 0);

    auto future = executor->run(*graph, std::make_shared<TaskComposerContext>(graph->getName()));
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
    EXPECT_EQ(future->context->isAborted(), false);
    EXPECT_EQ(future->context->isSuccessful(), true);
    EXPECT_EQ(future->context->task_infos->getInfoMap().size(), 6);
    EXPECT_TRUE(future->context->task_infos->getAbortingNode().is_nil());

    future->clear();
    EXPECT_FALSE(future->valid());

    // Serialization
    test_suite::runSerializationPointerTest(executor, "TaskComposerExecutorTests");
  }

  {  // Graph with child pipeline task conditional
    std::string str2 = R"(config:
                            conditional: false
                            nodes:
                              StartTask:
                                task: TestPipeline
                                config:
                                  conditional: true
                              DoneTask:
                                class: DoneTaskFactory
                                config:
                                  conditional: false
                              AbortTask:
                                class: DoneTaskFactory
                                config:
                                  conditional: false
                            edges:
                              - source: StartTask
                                destinations: [AbortTask, DoneTask]
                            terminals: [AbortTask, DoneTask])";
    YAML::Node config = YAML::Load(str2);
    auto graph = std::make_unique<TaskComposerGraph>("Graph", config["config"], factory);
    tesseract_planning::TaskComposerExecutor::UPtr executor = std::make_unique<T>(3);
    EXPECT_FALSE(executor->getName().empty());
    EXPECT_EQ(executor->getWorkerCount(), 3);
    EXPECT_EQ(executor->getTaskCount(), 0);

    auto future = executor->run(*graph, std::make_shared<TaskComposerContext>(graph->getName()));
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
    EXPECT_EQ(graph->getTerminals().size(), 2);
    auto task1 = graph->getNodeByName("StartTask");
    auto task2 = graph->getNodeByName("DoneTask");
    auto task3 = graph->getNodeByName("AbortTask");
    EXPECT_EQ(graph->getTerminals(), std::vector<boost::uuids::uuid>({ task3->getUUID(), task2->getUUID() }));
    EXPECT_EQ(task1->getInboundEdges().size(), 0);
    EXPECT_EQ(task1->getOutboundEdges().size(), 2);
    EXPECT_EQ(task1->getOutboundEdges().front(), task3->getUUID());
    EXPECT_EQ(task1->getOutboundEdges().back(), task2->getUUID());
    EXPECT_EQ(task2->getInboundEdges().size(), 1);
    EXPECT_EQ(task2->getInboundEdges().front(), task1->getUUID());
    EXPECT_EQ(task2->getOutboundEdges().size(), 0);
    EXPECT_EQ(task3->getInboundEdges().size(), 1);
    EXPECT_EQ(task3->getInboundEdges().front(), task1->getUUID());
    EXPECT_EQ(task3->getOutboundEdges().size(), 0);
    EXPECT_EQ(future->context->isAborted(), false);
    EXPECT_EQ(future->context->isSuccessful(), true);
    EXPECT_EQ(future->context->task_infos->getInfoMap().size(), 6);
    EXPECT_TRUE(future->context->task_infos->getAbortingNode().is_nil());

    future->clear();
    EXPECT_FALSE(future->valid());

    // Serialization
    test_suite::runSerializationPointerTest(executor, "TaskComposerExecutorTests");
  }

  {  // Graph with child graph task
    std::string str2 = R"(config:
                            conditional: false
                            nodes:
                              StartTask:
                                task: TestGraph
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
    YAML::Node config = YAML::Load(str2);
    auto graph = std::make_unique<TaskComposerGraph>("Graph", config["config"], factory);
    tesseract_planning::TaskComposerExecutor::UPtr executor = std::make_unique<T>("TaskComposerExecutorTests", 3);
    EXPECT_EQ(executor->getName(), "TaskComposerExecutorTests");
    EXPECT_EQ(executor->getWorkerCount(), 3);
    EXPECT_EQ(executor->getTaskCount(), 0);

    auto future = executor->run(*graph, std::make_shared<TaskComposerContext>(graph->getName()));
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
    EXPECT_EQ(future->context->isAborted(), false);
    EXPECT_EQ(future->context->isSuccessful(), true);
    EXPECT_EQ(future->context->task_infos->getInfoMap().size(), 6);
    EXPECT_TRUE(future->context->task_infos->getAbortingNode().is_nil());

    future->clear();
    EXPECT_FALSE(future->valid());

    // Serialization
    test_suite::runSerializationPointerTest(executor, "TaskComposerExecutorTests");
  }
}
}  // namespace tesseract_planning::test_suite

#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_EXECUTOR_UNIT_HPP
