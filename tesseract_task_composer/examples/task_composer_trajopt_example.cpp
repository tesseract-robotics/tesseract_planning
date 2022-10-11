
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <numeric>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/task_composer_graph.h>
#include <tesseract_task_composer/task_composer_data_storage.h>
#include <tesseract_task_composer/task_composer_utils.h>
#include <tesseract_task_composer/nodes/trajopt_motion_pipeline_task.h>
#include <tesseract_task_composer/taskflow/taskflow_task_composer_executor.h>

#include <tesseract_common/types.h>
#include <tesseract_environment/environment.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_visualization/visualization_loader.h>
#include <tesseract_support/tesseract_support_resource_locator.h>

#include "freespace_example_program.h"

using namespace tesseract_planning;

int main()
{
  // --------------------
  // Perform setup
  // --------------------
  auto locator = std::make_shared<tesseract_common::TesseractSupportResourceLocator>();
  tesseract_environment::Environment::Ptr env = std::make_shared<tesseract_environment::Environment>();
  tesseract_common::fs::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf");
  tesseract_common::fs::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf");
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

  // Define profiles
  auto profiles = std::make_shared<ProfileDictionary>();
  addDefaultPlannerProfiles(*profiles, { DEFAULT_PROFILE_KEY });
  addDefaultTaskComposerProfiles(*profiles, { DEFAULT_PROFILE_KEY });

  // Define the program
  CompositeInstruction program = freespaceExampleProgramIIWA();
  program.print();

  TaskComposerDataStorage task_data;
  task_data.setData("input_program", program);

  // Create problem
  TaskComposerProblem task_problem(env, task_data);

  // Create task input
  auto task_input = std::make_shared<TaskComposerInput>(task_problem, profiles);

  TaskComposerGraph::UPtr task_graph = std::make_unique<TrajOptMotionPipelineTask>("input_program", "output_program");

  std::ofstream tc_out_data;
  tc_out_data.open(tesseract_common::getTempPath() + "task_composer_trajopt_graph_example.dot");
  task_graph->dump(tc_out_data);  // dump the graph including dynamic tasks
  tc_out_data.close();

  auto task_executor = std::make_shared<TaskflowTaskComposerExecutor>();
  TaskComposerFuture::UPtr future = task_executor->run(*task_graph, *task_input);
  future->wait();

  // Plot Process Trajectory
  auto output_program = task_input->data_storage.getData("output_program").as<CompositeInstruction>();
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
