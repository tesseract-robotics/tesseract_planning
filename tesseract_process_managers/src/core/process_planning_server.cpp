/**
 * @file process_planning_server.cpp
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/task_info.h>
#include <tesseract_process_managers/core/process_planning_server.h>
#include <tesseract_process_managers/core/debug_observer.h>
#include <tesseract_process_managers/core/default_process_planners.h>

#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>
#include <tesseract_motion_planners/ompl/profile/ompl_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>
#include <tesseract_motion_planners/core/utils.h>

#include <tesseract_command_language/utils.h>

namespace tesseract_planning
{
ProcessPlanningServer::ProcessPlanningServer(EnvironmentCache::ConstPtr cache, size_t n) : cache_(std::move(cache))
{
  addExecutor(PRIMARY_EXECUTOR_NAME, n);
}

ProcessPlanningServer::ProcessPlanningServer(tesseract_environment::Environment::ConstPtr environment,
                                             int cache_size,
                                             size_t n)
  : cache_(std::make_shared<ProcessEnvironmentCache>(std::move(environment), cache_size))
{
  addExecutor(PRIMARY_EXECUTOR_NAME, n);
}

void ProcessPlanningServer::addExecutor(const std::string& name, const std::shared_ptr<tf::Executor>& executor)
{
  std::unique_lock lock(mutex_);
  executors_[name] = executor;
  executor->make_observer<DebugObserver>("ProcessPlanningObserver");
}

void ProcessPlanningServer::addExecutor(const std::string& name, size_t n)
{
  std::unique_lock lock(mutex_);
  auto executor = std::make_shared<tf::Executor>(n);
  executors_[name] = executor;
  /** @todo Need to figure out if these can associated with an individual run versus global */
  executor->make_observer<DebugObserver>("ProcessPlanningObserver");
}

bool ProcessPlanningServer::hasExecutor(const std::string& name) const
{
  std::shared_lock lock(mutex_);
  return (executors_.find(name) != executors_.end());
}

std::vector<std::string> ProcessPlanningServer::getAvailableExecutors() const
{
  std::shared_lock lock(mutex_);
  std::vector<std::string> executors;
  executors.reserve(process_planners_.size());
  for (const auto& executor : executors_)
    executors.push_back(executor.first);

  return executors;
}

void ProcessPlanningServer::registerProcessPlanner(const std::string& name, TaskflowGenerator::UPtr generator)
{
  std::unique_lock lock(mutex_);
  if (process_planners_.find(name) != process_planners_.end())
    CONSOLE_BRIDGE_logDebug("Process planner %s already exist so replacing with new generator.", name.c_str());

  process_planners_[name] = std::move(generator);
}

void ProcessPlanningServer::loadDefaultProcessPlanners()
{
  // This currently call registerProcessPlanner which takes a lock
  registerProcessPlanner(process_planner_names::TRAJOPT_PLANNER_NAME, createTrajOptGenerator());
#ifdef TESSERACT_PROCESS_MANAGERS_HAS_TRAJOPT_IFOPT
  registerProcessPlanner(process_planner_names::TRAJOPT_IFOPT_PLANNER_NAME, createTrajOptIfoptGenerator());
#endif
  registerProcessPlanner(process_planner_names::OMPL_PLANNER_NAME, createOMPLGenerator());
  registerProcessPlanner(process_planner_names::DESCARTES_PLANNER_NAME, createDescartesGenerator());
  registerProcessPlanner(process_planner_names::CARTESIAN_PLANNER_NAME, createCartesianGenerator());
  registerProcessPlanner(process_planner_names::FREESPACE_PLANNER_NAME, createFreespaceGenerator());
  registerProcessPlanner(process_planner_names::RASTER_FT_PLANNER_NAME, createRasterGenerator());
  registerProcessPlanner(process_planner_names::RASTER_O_FT_PLANNER_NAME, createRasterOnlyGenerator());
  registerProcessPlanner(process_planner_names::RASTER_G_FT_PLANNER_NAME, createRasterGlobalGenerator());
  registerProcessPlanner(process_planner_names::RASTER_FT_DT_PLANNER_NAME, createRasterDTGenerator());
  registerProcessPlanner(process_planner_names::RASTER_FT_WAAD_PLANNER_NAME, createRasterWAADGenerator());
  registerProcessPlanner(process_planner_names::RASTER_FT_WAAD_DT_PLANNER_NAME, createRasterWAADDTGenerator());
  registerProcessPlanner(process_planner_names::RASTER_O_G_FT_PLANNER_NAME, createRasterOnlyGlobalGenerator());
  registerProcessPlanner(process_planner_names::RASTER_CT_PLANNER_NAME, createRasterCTGenerator());
  registerProcessPlanner(process_planner_names::RASTER_O_CT_PLANNER_NAME, createRasterOnlyCTGenerator());
  registerProcessPlanner(process_planner_names::RASTER_CT_DT_PLANNER_NAME, createRasterCTDTGenerator());
  registerProcessPlanner(process_planner_names::RASTER_CT_WAAD_PLANNER_NAME, createRasterCTWAADGenerator());
  registerProcessPlanner(process_planner_names::RASTER_CT_WAAD_DT_PLANNER_NAME, createRasterCTWAADDTGenerator());
  registerProcessPlanner(process_planner_names::RASTER_G_CT_PLANNER_NAME, createRasterGlobalCTGenerator());
  registerProcessPlanner(process_planner_names::RASTER_O_G_CT_PLANNER_NAME, createRasterOnlyGlobalCTGenerator());
}

bool ProcessPlanningServer::hasProcessPlanner(const std::string& name) const
{
  std::shared_lock lock(mutex_);
  return (process_planners_.find(name) != process_planners_.end());
}

std::vector<std::string> ProcessPlanningServer::getAvailableProcessPlanners() const
{
  std::shared_lock lock(mutex_);
  std::vector<std::string> planners;
  planners.reserve(process_planners_.size());
  for (const auto& planner : process_planners_)
    planners.push_back(planner.first);

  return planners;
}

ProcessPlanningFuture ProcessPlanningServer::run(const ProcessPlanningRequest& request) const
{
  CONSOLE_BRIDGE_logDebug("Tesseract Planning Server Received Request!");

  auto problem = std::make_shared<ProcessPlanningProblem>();
  problem->name = request.name;
  problem->plan_profile_remapping = std::make_unique<const PlannerProfileRemapping>(request.plan_profile_remapping);
  problem->composite_profile_remapping =
      std::make_unique<const PlannerProfileRemapping>(request.composite_profile_remapping);

  problem->input = std::make_unique<InstructionPoly>(request.instructions);
  const auto& composite_program = problem->input->as<CompositeInstruction>();
  tesseract_common::ManipulatorInfo mi = composite_program.getManipulatorInfo();
  problem->global_manip_info = std::make_unique<const tesseract_common::ManipulatorInfo>(mi);

  if (!request.seed.isNull())
    problem->results = std::make_unique<InstructionPoly>(request.seed);

  // Assign the problems environment
  tesseract_environment::Environment::Ptr tc = cache_->getCachedEnvironment();

  // Set the env state if provided
  if (!request.env_state.joints.empty())
    tc->setState(request.env_state.joints);

  if (!request.commands.empty() && !tc->applyCommands(request.commands))
  {
    CONSOLE_BRIDGE_logError("Tesseract Planning Server failed to apply commands to environment!");
    ProcessPlanningFuture response;
    response.problem = problem;
    return response;
  }
  problem->env = tc;

  return run(problem, request.executor_name, request.save_io);
}

ProcessPlanningFuture ProcessPlanningServer::run(ProcessPlanningProblem::Ptr problem,
                                                 const std::string& name,
                                                 bool save_io) const
{
  CONSOLE_BRIDGE_logDebug("Tesseract Planning Server Received Problem!");
  ProcessPlanningFuture response;
  response.problem = std::move(problem);

  auto it = process_planners_.find(response.problem->name);
  if (it == process_planners_.end())
  {
    CONSOLE_BRIDGE_logError("Requested motion Process Pipeline (aka. Taskflow) is not supported!");
    return response;
  }

  std::shared_ptr<tf::Executor> executor;
  {
    std::shared_lock lock(mutex_);
    auto it = executors_.find(name);
    if (it == executors_.end())
    {
      CONSOLE_BRIDGE_logError("Requested executor '%s' does not exist!", name.c_str());
      return response;
    }

    executor = it->second;
  }

  // This makes sure the Joint and State Waypoints match the same order as the kinematics
  auto& composite_program = response.problem->input->as<CompositeInstruction>();
  if (formatProgram(composite_program, *response.problem->env))
  {
    CONSOLE_BRIDGE_logDebug("Tesseract Planning Server: Input program required formatting!");
  }

  bool has_seed{ true };
  if (response.problem->results == nullptr || response.problem->results->isNull())
  {
    has_seed = false;
    response.problem->results = std::make_unique<InstructionPoly>(generateSkeletonSeed(composite_program));
  }

  // Create Task input
  TaskInput task_input(response.problem->env,
                       response.problem->input.get(),
                       *(response.problem->global_manip_info),
                       *(response.problem->plan_profile_remapping),
                       *(response.problem->composite_profile_remapping),
                       response.problem->results.get(),
                       has_seed,
                       profiles_);
  task_input.save_io = save_io;
  response.interface = task_input.getTaskInterface();
  response.problem->taskflow_container = it->second->generateTaskflow(task_input, nullptr, nullptr);

  // Dump taskflow graph before running
  if (console_bridge::getLogLevel() == console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG)
  {
    std::ofstream out_data;
    out_data.open(tesseract_common::getTempPath() + response.problem->name + "-" +
                  tesseract_common::getTimestampString() + ".dot");
    response.problem->taskflow_container.taskflow->dump(out_data);
    out_data.close();
  }

  tf::Future<void> fu = executor->run(*(response.problem->taskflow_container.taskflow));
  response.process_future = fu.share();
  CONSOLE_BRIDGE_logDebug("Tesseract Planning Server Finished!");
  return response;
}

tf::Future<void> ProcessPlanningServer::run(tf::Taskflow& taskflow, const std::string& name) const
{
  std::shared_ptr<tf::Executor> executor;
  {
    std::shared_lock lock(mutex_);
    executor = executors_.at(name);
  }
  return executor->run(taskflow);
}

void ProcessPlanningServer::waitForAll(const std::string& name) const
{
  std::shared_ptr<tf::Executor> executor;
  {
    std::shared_lock lock(mutex_);
    executor = executors_.at(name);
  }
  executor->wait_for_all();
}

void ProcessPlanningServer::enableTaskflowProfiling(const std::string& name)
{
  std::unique_lock lock(mutex_);
  auto it = profile_observers_.find(name);
  if (it == profile_observers_.end())
    profile_observers_[name] = executors_.at(name)->make_observer<tf::TFProfObserver>();
}

void ProcessPlanningServer::disableTaskflowProfiling(const std::string& name)
{
  std::unique_lock lock(mutex_);
  auto it = profile_observers_.find(name);
  if (it != profile_observers_.end())
  {
    executors_.at(name)->remove_observer(it->second);
    profile_observers_.erase(name);
  }
}

ProfileDictionary::Ptr ProcessPlanningServer::getProfiles()
{
  std::shared_lock lock(mutex_);
  return profiles_;
}

ProfileDictionary::ConstPtr ProcessPlanningServer::getProfiles() const
{
  std::shared_lock lock(mutex_);
  return profiles_;
}

long ProcessPlanningServer::getWorkerCount(const std::string& name) const
{
  std::shared_ptr<tf::Executor> executor;
  {
    std::shared_lock lock(mutex_);
    executor = executors_.at(name);
  }

  return static_cast<long>(executor->num_workers());
}

long ProcessPlanningServer::getTaskCount(const std::string& name) const
{
  std::shared_ptr<tf::Executor> executor;
  {
    std::shared_lock lock(mutex_);
    executor = executors_.at(name);
  }
  return static_cast<long>(executor->num_topologies());
}

}  // namespace tesseract_planning
