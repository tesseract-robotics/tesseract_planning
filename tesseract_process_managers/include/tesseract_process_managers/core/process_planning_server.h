/**
 * @file process_planning_server.h
 * @brief A process planning server with a default set of process planners
 *
 * @author Levi Armstrong
 * @date August 18, 2020
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
#ifndef TESSERACT_PROCESS_MANAGERS_PROCESS_PLANNING_SERVER_H
#define TESSERACT_PROCESS_MANAGERS_PROCESS_PLANNING_SERVER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <string>
#include <vector>
#include <shared_mutex>
#include <taskflow/taskflow.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/profile_dictionary.h>

#include <tesseract_process_managers/core/process_environment_cache.h>
#include <tesseract_process_managers/core/taskflow_generator.h>
#include <tesseract_process_managers/core/process_planning_request.h>
#include <tesseract_process_managers/core/process_planning_future.h>

namespace tesseract_planning
{
/**
 * @brief A process planning server that support asynchronous execution of process planning requests
 * @details It allows the developer to register Process pipelines (aka. Taskflow Generators) so they may be request
 * @note This class is thread safe
 */
class ProcessPlanningServer
{
public:
  using Ptr = std::shared_ptr<ProcessPlanningServer>;
  using ConstPtr = std::shared_ptr<const ProcessPlanningServer>;
  using UPtr = std::unique_ptr<ProcessPlanningServer>;
  using ConstUPtr = std::unique_ptr<const ProcessPlanningServer>;

  /**
   * @brief Constructor
   * @param cache The cache to use for getting Environment objects
   * @param n The number of threads used by the planning server
   */
  ProcessPlanningServer(EnvironmentCache::ConstPtr cache, size_t n = std::thread::hardware_concurrency());

  /**
   * @brief Constructor
   * @param environment The environment object to leverage
   * @param cache_size The cache size used for maintaining a que of environments for improved performance when making
   * multiple requests
   * @param n The number of threads used by the planning server
   */
  ProcessPlanningServer(tesseract_environment::Environment::ConstPtr environment,
                        int cache_size = 1,
                        size_t n = std::thread::hardware_concurrency());

  virtual ~ProcessPlanningServer() = default;

  ProcessPlanningServer(const ProcessPlanningServer&) = delete;
  ProcessPlanningServer& operator=(const ProcessPlanningServer&) = delete;
  ProcessPlanningServer(ProcessPlanningServer&&) = delete;
  ProcessPlanningServer& operator=(ProcessPlanningServer&&) = delete;

  /**
   * @brief Add a executors (thread pool) under the provided name
   * @param name The name of the thread pool
   * @param executor The executor to add
   */
  void addExecutor(const std::string& name, const std::shared_ptr<tf::Executor>& executor);

  /**
   * @brief Add a executors (thread pool) under the provided name
   * @details This creates a taskflow executor with the provided number of threads
   * @param name The name of the thread pool
   * @param n The number of threads
   */
  void addExecutor(const std::string& name, size_t n);

  /**
   * @brief Check if executors (thread pool) exists with the provided name
   * @param name The name to search
   * @return True if it exists, otherwise false
   */
  bool hasExecutor(const std::string& name) const;

  /**
   * @brief Get the available executors (thread pool) names
   * @return A vector of names
   */
  std::vector<std::string> getAvailableExecutors() const;

  /**
   * @brief Register a process planner with the planning server
   * @param name The name used to locate the process planner through requests
   * @param generator The Taskflow Generator associated with the name
   */
  void registerProcessPlanner(const std::string& name, TaskflowGenerator::UPtr generator);

  /**
   * @brief Load default process planners
   * @details This is not called automatically, so user but call this to load default planners.
   */
  void loadDefaultProcessPlanners();

  /**
   * @brief Check if the planning server has a name associated with a process pipeline
   * @param name The name of the process planner to check for
   * @return True if the name is already taken, otherwise false
   */
  bool hasProcessPlanner(const std::string& name) const;

  /**
   * @brief Get a list of process planner registered with the planning server
   * @return A vector of names
   */
  std::vector<std::string> getAvailableProcessPlanners() const;

  /**
   * @brief Execute a process planning request.
   * @details This does not block to allow for multiple requests, use future to wait if needed.
   * @param request The process planning request to execute
   * @return A process planning future to get results and monitor the execution along with the ability to abort
   */
  ProcessPlanningFuture run(const ProcessPlanningRequest& request) const;

  /**
   * @brief Execute a process planning problem
   * @details This does not block to allow for multiple requests, use future to wait if needed.
   * @note This is primarily used for replanning cached plans
   * @param problem The problem to solve
   * @param name The name of the executor to use
   * @param save_io Indicate if tasks should store input and output results in the task info
   * @return A process planning future to get results and monitor the execution along with the ability to abort
   */
  ProcessPlanningFuture run(ProcessPlanningProblem::Ptr problem,
                            const std::string& name = PRIMARY_EXECUTOR_NAME,
                            bool save_io = false) const;

  /**
   * @brief This is a utility function to run arbitrary taskflows
   * @param taskflow The taskflow to execute
   * @param name The name of the executor to use
   * @return A future to monitor progress
   */
  tf::Future<void> run(tf::Taskflow& taskflow, const std::string& name = PRIMARY_EXECUTOR_NAME) const;

  /**
   * @brief Wait for all process currently being executed to finish before returning
   * @param name The name of the executor to wait on
   */
  void waitForAll(const std::string& name = PRIMARY_EXECUTOR_NAME) const;

  /**
   * @brief This add a Taskflow profiling observer to the executor
   * @param name The name of the executor to enable profiling for
   */
  void enableTaskflowProfiling(const std::string& name = PRIMARY_EXECUTOR_NAME);

  /**
   * @brief This remove the Taskflow profiling observer from the executor if one exists
   * @param name The name of the executor to disable profiling for
   */
  void disableTaskflowProfiling(const std::string& name = PRIMARY_EXECUTOR_NAME);

  /**
   * @brief Get the profile dictionary associated with the planning server
   * @return Profile dictionary
   */
  ProfileDictionary::Ptr getProfiles();

  /**
   * @brief Get the profile dictionary associated with the planning server (const)
   * @return Profile dictionary (const)
   */
  ProfileDictionary::ConstPtr getProfiles() const;

  /**
   * @brief Queries the number of worker threads (can be zero)
   * @param name The name of the executor to get worker count for
   */
  long getWorkerCount(const std::string& name = PRIMARY_EXECUTOR_NAME) const;

  /**
   * @brief Queries the number of running tasks at the time of this call
   * When a taskflow is submitted to an executor, a topology is created to store
   * runtime metadata of the running taskflow.
   * @param name The name of the executor to get task count for
   */
  long getTaskCount(const std::string& name = PRIMARY_EXECUTOR_NAME) const;

protected:
  mutable std::shared_mutex mutex_;

  /** @brief An environment cache which is thread safe */
  EnvironmentCache::ConstPtr cache_;

  /** @brief A map of executors which are thread safe */
  std::unordered_map<std::string, std::shared_ptr<tf::Executor>> executors_;

  /** @brief A map of profile observers */
  std::unordered_map<std::string, std::shared_ptr<tf::TFProfObserver>> profile_observers_;

  /** @brief A map of process planners */
  std::unordered_map<std::string, TaskflowGenerator::UPtr> process_planners_;

  /** @brief The profile dictionary which is thread safe */
  ProfileDictionary::Ptr profiles_{ std::make_shared<ProfileDictionary>() };
};

}  // namespace tesseract_planning

#endif  // TESSERACT_PROCESS_MANAGERS_PROCESS_PLANNING_SERVER_H
