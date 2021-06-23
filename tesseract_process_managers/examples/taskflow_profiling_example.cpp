/**
 * @file taskflow_profiling_example.cpp
 * @brief Demonstrates tools for profiling taskflows using the task infos
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date June 23. 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#include <boost/serialization/vector.hpp>
#include <taskflow/taskflow.hpp>
#include <fstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/ofkt/ofkt_state_solver.h>
#include <tesseract_command_language/command_language.h>
#include <freespace_example_program.h>
#include <tesseract_process_managers/utils/task_info_statistics.h>

#include <tesseract_command_language/utils/utils.h>
#include <tesseract_process_managers/taskflow_generators/freespace_taskflow.h>
#include <tesseract_process_managers/core/process_planning_server.h>
#include <tesseract_process_managers/utils/task_info_statistics.h>
#include <tesseract_visualization/visualization_loader.h>

using namespace tesseract_planning;

std::string locateResource(const std::string& url)
{
  std::string mod_url = url;
  if (url.find("package://tesseract_support") == 0)
  {
    mod_url.erase(0, strlen("package://tesseract_support"));
    size_t pos = mod_url.find('/');
    if (pos == std::string::npos)
    {
      return std::string();
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = std::string(TESSERACT_SUPPORT_DIR);

    if (package_path.empty())
    {
      return std::string();
    }

    mod_url = package_path + mod_url;
  }

  return mod_url;
}

int main()
{
  // --------------------
  // Perform setup
  // --------------------
  tesseract_scene_graph::ResourceLocator::Ptr locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
  tesseract_environment::Environment::Ptr env = std::make_shared<tesseract_environment::Environment>();
  tesseract_common::fs::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf");
  tesseract_common::fs::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf");
  env->init<tesseract_environment::OFKTStateSolver>(urdf_path, srdf_path, locator);

  // Dynamically load ignition visualizer if exist
  tesseract_visualization::VisualizationLoader loader;
  auto plotter = loader.get();

  if (plotter != nullptr)
  {
    plotter->waitForConnection(3);
    if (plotter->isConnected())
      plotter->plotEnvironment(env);
  }

  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env), 1);
  planning_server.loadDefaultProcessPlanners();

  for (int idx = -10; idx < 10; idx++)
  {
    // Create Process Planning Request
    ProcessPlanningRequest request;
    request.name = process_planner_names::FREESPACE_PLANNER_NAME;

    // Define the program
    CompositeInstruction program = freespaceExampleProgramIIWA(
        Eigen::Isometry3d::Identity() * Eigen::Translation3d(static_cast<double>(idx) / 10., 0.2, 1.0));
    request.instructions = Instruction(program);

    // Solve process plan
    ProcessPlanningFuture response = planning_server.run(request);
    planning_server.waitForAll();

    // Plot Process Trajectory
    if (plotter != nullptr && plotter->isConnected())
    {
      plotter->waitForInput();
      plotter->plotTrajectory(toJointTrajectory(response.results->as<CompositeInstruction>()), env->getStateSolver());
    }

    // Convert to an easily loggable vector
    auto info_map = response.interface->getTaskInfoMap();
    std::vector<TaskInfo> task_info_vec;
    for (const auto& info : info_map)
      task_info_vec.push_back(*info.second);

    // Save to the temp directory
    std::string log_dir = tesseract_common::getTempPath() + "/task_infos/";
    boost::filesystem::create_directories(log_dir);
    std::string file_path = log_dir + "task_info_" + tesseract_common::getTimestampString() + ".xml";
    Serialization::toArchiveFileXML<std::vector<TaskInfo>>(task_info_vec, file_path);
  }

  std::cout << "Execution Complete" << std::endl;

  // Load task infos
  TaskInfoProfiler profiler;
  profiler.load(tesseract_common::getTempPath() + "task_infos");

  // Print summary
  profiler.print();

  // Dump Graphviz file including statistics
  auto display_info = profiler.getTaskDisplayInfo();

  /// @todo When FreespaceTaskflow is rewritten to inherit from GraphTaskflow this will work.
  //  std::ofstream out_data;
  //  out_data.open(tesseract_common::getTempPath() + "profiling_taskflow-" + tesseract_common::getTimestampString() +
  //                ".dot");
  //  FreespaceTaskflowParams params;
  //  auto graph_taskflow = std::make_unique<FreespaceTaskflow>(params);
  //  dump(out_data, graph_taskflow, display_info);
  //  out_data.close();

  return 0;
}
