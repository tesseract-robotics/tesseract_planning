
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <taskflow/taskflow.hpp>
#include <fstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "raster_example_program.h"

#include <tesseract_task_composer/task_composer_graph.h>
#include <tesseract_task_composer/task_composer_data_storage.h>
#include <tesseract_task_composer/task_composer_plugin_factory.h>

#include <tesseract_common/types.h>
#include <tesseract_environment/environment.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_visualization/visualization_loader.h>
//#include <tesseract_process_managers/utils/task_info_statistics.h>
#include <tesseract_support/tesseract_support_resource_locator.h>

using namespace tesseract_planning;

int main()
{
  // --------------------
  // Perform setup
  // --------------------
  auto locator = std::make_shared<tesseract_common::TesseractSupportResourceLocator>();
  auto env = std::make_shared<tesseract_environment::Environment>();
  tesseract_common::fs::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400.urdf");
  tesseract_common::fs::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400.srdf");
  env->init(urdf_path, srdf_path, locator);

  // Dynamically load ignition visualizer if exist
  tesseract_visualization::VisualizationLoader loader;
  auto plotter = loader.get();

  if (plotter != nullptr)
  {
    plotter->waitForConnection(3);
    if (plotter->isConnected())
      plotter->plotEnvironment(*env);
  }
  // Get plugin factory
  const std::string share_dir(TESSERACT_TASK_COMPOSER_DIR);
  tesseract_common::fs::path config_path(share_dir + "/config/task_composer_plugins.yaml");
  TaskComposerPluginFactory factory(config_path);

  // Create raster task
  TaskComposerNode::UPtr task = factory.createTaskComposerNode("RasterFtPipeline");
  const std::string input_key = task->getInputKeys().front();
  const std::string output_key = task->getOutputKeys().front();

  // Define profiles
  auto profiles = std::make_shared<ProfileDictionary>();

  // Define the program
  CompositeInstruction program = rasterExampleProgram();
  program.print();

  // Create data storage
  TaskComposerDataStorage task_data;
  task_data.setData(input_key, program);

  // Create problem
  TaskComposerProblem task_problem(env, task_data);

  // Create task input
  auto task_input = std::make_shared<TaskComposerInput>(task_problem, profiles);

  // Save dot graph
  std::ofstream tc_out_data;
  tc_out_data.open(tesseract_common::getTempPath() + "task_composer_raster_example.dot");
  task->dump(tc_out_data);  // dump the graph including dynamic tasks
  tc_out_data.close();

  // Solve raster plan
  auto task_executor = factory.createTaskComposerExecutor("TaskflowExecutor");
  TaskComposerFuture::UPtr future = task_executor->run(*task, *task_input);
  future->wait();

  // Plot Process Trajectory
  auto output_program = task_input->data_storage.getData(output_key).as<CompositeInstruction>();
  if (plotter != nullptr && plotter->isConnected())
  {
    plotter->waitForInput();
    plotter->plotTrajectory(toJointTrajectory(output_program), *env->getStateSolver());
  }

  std::cout << "Execution Complete" << std::endl;

  //  // Print summary statistics
  //  std::map<std::size_t, TaskInfo::UPtr> info_map = response.interface->getTaskInfoMap();
  //  TaskInfoProfiler profiler;
  //  profiler.load(info_map);
  //  profiler.print();

  return 0;
}
