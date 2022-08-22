/**
 * @file task_composer_executor.h
 * @brief The executor for executing task graphs
 *
 * @author Levi Armstrong
 * @date July 29. 2022
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2022, Levi Armstrong
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
#ifndef TESSERACT_TASK_COMPOSER_TASK_COMPOSER_EXECUTOR_H
#define TESSERACT_TASK_COMPOSER_TASK_COMPOSER_EXECUTOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/task_composer_graph.h>
#include <tesseract_task_composer/task_composer_task.h>
#include <tesseract_task_composer/task_composer_input.h>
#include <tesseract_task_composer/task_composer_future.h>

namespace tesseract_planning
{
class TaskComposerExecutor
{
public:
  using Ptr = std::shared_ptr<TaskComposerExecutor>;
  using ConstPtr = std::shared_ptr<const TaskComposerExecutor>;
  using UPtr = std::unique_ptr<TaskComposerExecutor>;
  using ConstUPtr = std::unique_ptr<const TaskComposerExecutor>;

  TaskComposerExecutor(std::string name = "TaskComposerExecutor");
  virtual ~TaskComposerExecutor() = default;

  /**
   * @brief Execute the provided task graph
   * @param task_graph The task graph to execute
   * @param task_input The task input provided to every task
   * @return The future associated with execution
   */
  virtual TaskComposerFuture::UPtr run(const TaskComposerGraph& task_graph, TaskComposerInput& task_input) = 0;

  /**
   * @brief Execute the provided task
   * @param task_graph The task to execute
   * @param task_input The task input provided to task
   * @return The future associated with execution
   */
  virtual TaskComposerFuture::UPtr run(const TaskComposerTask& task, TaskComposerInput& task_input) = 0;

  /** @brief Queries the number of workers (example: number of threads) */
  virtual long getWorkerCount() const = 0;

  /** @brief Queries the number of running tasks at the time of this call */
  virtual long getTaskCount() const = 0;

  bool operator==(const TaskComposerExecutor& rhs) const;
  bool operator!=(const TaskComposerExecutor& rhs) const;

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  std::string name_;
};
}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::TaskComposerExecutor, "TaskComposerExecutor")

// class Executor {

//  friend class FlowBuilder;
//  friend class Subflow;
//  friend class cudaFlow;

//  struct PerThread {
//    Worker* worker;
//    PerThread() : worker {nullptr} { }
//  };

//  public:

//    /**
//    @brief constructs the executor with N worker threads
//    */
//    explicit Executor(size_t N = std::thread::hardware_concurrency());

//    /**
//    @brief destructs the executor
//    */
//    ~Executor();

//    /**
//    @brief runs the taskflow once

//    @param taskflow a tf::Taskflow object

//    @return a tf::Future that will holds the result of the execution
//    */
//    tf::Future<void> run(Taskflow& taskflow);

//    /**
//    @brief runs the taskflow once and invoke a callback upon completion

//    @param taskflow a tf::Taskflow object
//    @param callable a callable object to be invoked after this run

//    @return a tf::Future that will holds the result of the execution
//    */
//    template<typename C>
//    tf::Future<void> run(Taskflow& taskflow, C&& callable);

//    /**
//    @brief runs the taskflow for N times

//    @param taskflow a tf::Taskflow object
//    @param N number of runs

//    @return a tf::Future that will holds the result of the execution
//    */
//    tf::Future<void> run_n(Taskflow& taskflow, size_t N);

//    /**
//    @brief runs the taskflow for N times and then invokes a callback

//    @param taskflow a tf::Taskflow
//    @param N number of runs
//    @param callable a callable object to be invoked after this run

//    @return a tf::Future that will holds the result of the execution
//    */
//    template<typename C>
//    tf::Future<void> run_n(Taskflow& taskflow, size_t N, C&& callable);

//    /**
//    @brief runs the taskflow multiple times until the predicate becomes true and
//           then invokes a callback

//    @param taskflow a tf::Taskflow
//    @param pred a boolean predicate to return true for stop

//    @return a tf::Future that will holds the result of the execution
//    */
//    template<typename P>
//    tf::Future<void> run_until(Taskflow& taskflow, P&& pred);

//    /**
//    @brief runs the taskflow multiple times until the predicate becomes true and
//           then invokes the callback

//    @param taskflow a tf::Taskflow
//    @param pred a boolean predicate to return true for stop
//    @param callable a callable object to be invoked after this run

//    @return a tf::Future that will holds the result of the execution
//    */
//    template<typename P, typename C>
//    tf::Future<void> run_until(Taskflow& taskflow, P&& pred, C&& callable);

//    /**
//    @brief wait for all pending graphs to complete
//    */
//    void wait_for_all();

//    /**
//    @brief queries the number of worker threads (can be zero)
//    */
//    size_t num_workers() const;

//    /**
//    @brief queries the number of running topologies at the time of this call

//    When a taskflow is submitted to an executor, a topology is created to store
//    runtime metadata of the running taskflow.
//    */
//    size_t num_topologies() const;

//    /**
//    @brief queries the id of the caller thread in this executor

//    Each worker has an unique id from 0 to N-1 exclusive to the associated executor.
//    If the caller thread does not belong to the executor, -1 is returned.
//    */
//    int this_worker_id() const;

//    /**
//    @brief runs a given function asynchronously

//    @tparam F callable type
//    @tparam ArgsT parameter types

//    @param f callable object to call
//    @param args parameters to pass to the callable

//    @return a tf::Future that will holds the result of the execution

//    This method is thread-safe. Multiple threads can launch asynchronous tasks
//    at the same time.
//    */
//    template <typename F, typename... ArgsT>
//    auto async(F&& f, ArgsT&&... args);

//    /**
//    @brief similar to tf::Executor::async but does not return a future object
//    */
//    template <typename F, typename... ArgsT>
//    void silent_async(F&& f, ArgsT&&... args);

//    /**
//    @brief constructs an observer to inspect the activities of worker threads

//    Each executor manage a list of observers in shared ownership with callers.

//    @tparam Observer observer type derived from tf::ObserverInterface
//    @tparam ArgsT argument parameter pack

//    @param args arguments to forward to the constructor of the observer

//    @return a shared pointer to the created observer
//    */
//    template <typename Observer, typename... ArgsT>
//    std::shared_ptr<Observer> make_observer(ArgsT&&... args);

//    /**
//    @brief removes the associated observer
//    */
//    template <typename Observer>
//    void remove_observer(std::shared_ptr<Observer> observer);

//    /**
//    @brief queries the number of observers
//    */
//    size_t num_observers() const;
#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_EXECUTOR_H
