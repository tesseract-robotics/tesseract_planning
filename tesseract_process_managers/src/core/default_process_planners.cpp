/**
 * @file default_process_planners.h
 * @brief The default process planners provided by Tesseract
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

#include <tesseract_process_managers/core/default_process_planners.h>
#include <tesseract_process_managers/taskflow_generators/raster_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_global_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_only_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_only_global_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_dt_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_waad_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_waad_dt_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/graph_taskflow.h>

#include <tesseract_process_managers/task_generators/has_seed_task_generator.h>
#include <tesseract_process_managers/task_generators/motion_planner_task_generator.h>
#include <tesseract_process_managers/task_generators/seed_min_length_task_generator.h>
#include <tesseract_process_managers/task_generators/discrete_contact_check_task_generator.h>
#include <tesseract_process_managers/task_generators/iterative_spline_parameterization_task_generator.h>
#include <tesseract_process_managers/task_generators/ruckig_trajectory_smoothing_task_generator.h>
#include <tesseract_process_managers/task_generators/check_input_task_generator.h>

#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#ifdef TESSERACT_PROCESS_MANAGERS_HAS_TRAJOPT_IFOPT
#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_motion_planner.h>
#endif
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>

namespace tesseract_planning
{
TaskflowGenerator::UPtr createTrajOptGenerator(bool check_input, bool post_collision_check, bool post_smoothing)
{
  auto tf = std::make_unique<GraphTaskflow>("TrajOptTaskflow");

  int check_input_task{ std::numeric_limits<int>::min() };
  if (check_input)
    check_input_task = tf->addNode(std::make_unique<CheckInputTaskGenerator>(), true);

  // Check if seed was provided
  int has_seed_task = tf->addNode(std::make_unique<HasSeedTaskGenerator>(), true);

  // Simple planner as interpolator
  auto interpolator = std::make_shared<SimpleMotionPlanner>();
  int interpolator_task = tf->addNode(std::make_unique<MotionPlannerTaskGenerator>(interpolator), true);

  // Setup Seed Min Length Process Generator
  // This is required because trajopt requires a minimum length trajectory. This is used to correct the seed if it is
  // to short.
  int seed_min_length_task = tf->addNode(std::make_unique<SeedMinLengthTaskGenerator>());

  // Setup TrajOpt
  auto motion_planner = std::make_shared<TrajOptMotionPlanner>();
  int motion_planner_task = tf->addNode(std::make_unique<MotionPlannerTaskGenerator>(motion_planner), true);

  // Setup post collision check
  int contact_check_task{ std::numeric_limits<int>::min() };
  if (post_collision_check)
    contact_check_task = tf->addNode(std::make_unique<DiscreteContactCheckTaskGenerator>(), true);

  // Setup time parameterization
  int time_parameterization_task = tf->addNode(std::make_unique<IterativeSplineParameterizationTaskGenerator>(), true);

  // Setup trajectory smoothing
  int smoothing_task{ std::numeric_limits<int>::min() };
  if (post_smoothing)
    smoothing_task = tf->addNode(std::make_unique<RuckigTrajectorySmoothingTaskGenerator>(), true);

  if (check_input)
    tf->addEdges(check_input_task, { GraphTaskflow::ERROR_NODE, has_seed_task });

  tf->addEdges(has_seed_task, { interpolator_task, seed_min_length_task });
  tf->addEdges(interpolator_task, { GraphTaskflow::ERROR_NODE, seed_min_length_task });
  tf->addEdges(seed_min_length_task, { motion_planner_task });

  if (post_collision_check)
  {
    tf->addEdges(motion_planner_task, { GraphTaskflow::ERROR_NODE, contact_check_task });
    tf->addEdges(contact_check_task, { GraphTaskflow::ERROR_NODE, time_parameterization_task });
  }
  else
  {
    tf->addEdges(motion_planner_task, { GraphTaskflow::ERROR_NODE, time_parameterization_task });
  }

  if (post_smoothing)
  {
    tf->addEdges(time_parameterization_task, { GraphTaskflow::ERROR_NODE, smoothing_task });
    tf->addEdges(smoothing_task, { GraphTaskflow::DONE_NODE });
  }
  else
  {
    tf->addEdges(time_parameterization_task, { GraphTaskflow::ERROR_NODE, GraphTaskflow::DONE_NODE });
  }

  return tf;
}

#ifdef TESSERACT_PROCESS_MANAGERS_HAS_TRAJOPT_IFOPT
TaskflowGenerator::UPtr createTrajOptIfoptGenerator(bool check_input, bool post_collision_check, bool post_smoothing)
{
  auto tf = std::make_unique<GraphTaskflow>("TrajOptIfoptTaskflow");

  int check_input_task{ std::numeric_limits<int>::min() };
  if (check_input)
    check_input_task = tf->addNode(std::make_unique<CheckInputTaskGenerator>(), true);

  // Check if seed was provided
  int has_seed_task = tf->addNode(std::make_unique<HasSeedTaskGenerator>(), true);

  // Simple planner as interpolator
  auto interpolator = std::make_shared<SimpleMotionPlanner>();
  int interpolator_task = tf->addNode(std::make_unique<MotionPlannerTaskGenerator>(interpolator), true);

  // Setup Seed Min Length Process Generator
  // This is required because trajopt requires a minimum length trajectory. This is used to correct the seed if it is
  // to short.
  int seed_min_length_task = tf->addNode(std::make_unique<SeedMinLengthTaskGenerator>());

  // Setup TrajOpt IFOPT
  auto motion_planner = std::make_shared<TrajOptIfoptMotionPlanner>();
  int motion_planner_task = tf->addNode(std::make_unique<MotionPlannerTaskGenerator>(motion_planner), true);

  // Setup post collision check
  int contact_check_task{ std::numeric_limits<int>::min() };
  if (post_collision_check)
    contact_check_task = tf->addNode(std::make_unique<DiscreteContactCheckTaskGenerator>(), true);

  // Setup time parameterization
  int time_parameterization_task = tf->addNode(std::make_unique<IterativeSplineParameterizationTaskGenerator>(), true);

  // Setup trajectory smoothing
  int smoothing_task{ std::numeric_limits<int>::min() };
  if (post_smoothing)
    smoothing_task = tf->addNode(std::make_unique<RuckigTrajectorySmoothingTaskGenerator>(), true);

  if (check_input)
    tf->addEdges(check_input_task, { GraphTaskflow::ERROR_NODE, has_seed_task });

  tf->addEdges(has_seed_task, { interpolator_task, seed_min_length_task });
  tf->addEdges(interpolator_task, { GraphTaskflow::ERROR_NODE, seed_min_length_task });
  tf->addEdges(seed_min_length_task, { motion_planner_task });

  if (post_collision_check)
  {
    tf->addEdges(motion_planner_task, { GraphTaskflow::ERROR_NODE, contact_check_task });
    tf->addEdges(contact_check_task, { GraphTaskflow::ERROR_NODE, time_parameterization_task });
  }
  else
  {
    tf->addEdges(motion_planner_task, { GraphTaskflow::ERROR_NODE, time_parameterization_task });
  }

  if (post_smoothing)
  {
    tf->addEdges(time_parameterization_task, { GraphTaskflow::ERROR_NODE, smoothing_task });
    tf->addEdges(smoothing_task, { GraphTaskflow::DONE_NODE });
  }
  else
  {
    tf->addEdges(time_parameterization_task, { GraphTaskflow::ERROR_NODE, GraphTaskflow::DONE_NODE });
  }

  return tf;
}
#endif

TaskflowGenerator::UPtr createOMPLGenerator(bool check_input, bool post_collision_check, bool post_smoothing)
{
  auto tf = std::make_unique<GraphTaskflow>("OMPLTaskflow");

  int check_input_task{ std::numeric_limits<int>::min() };
  if (check_input)
    check_input_task = tf->addNode(std::make_unique<CheckInputTaskGenerator>(), true);

  // Check if seed was provided
  int has_seed_task = tf->addNode(std::make_unique<HasSeedTaskGenerator>(), true);

  // Simple planner as interpolator
  auto interpolator = std::make_shared<SimpleMotionPlanner>();
  int interpolator_task = tf->addNode(std::make_unique<MotionPlannerTaskGenerator>(interpolator), true);

  // Setup Seed Min Length Process Generator
  // This is required because trajopt requires a minimum length trajectory. This is used to correct the seed if it is
  // to short.
  int seed_min_length_task = tf->addNode(std::make_unique<SeedMinLengthTaskGenerator>());

  // Setup OMPL
  auto motion_planner = std::make_shared<OMPLMotionPlanner>();
  int motion_planner_task = tf->addNode(std::make_unique<MotionPlannerTaskGenerator>(motion_planner), true);

  // Setup post collision check
  int contact_check_task{ std::numeric_limits<int>::min() };
  if (post_collision_check)
    contact_check_task = tf->addNode(std::make_unique<DiscreteContactCheckTaskGenerator>(), true);

  // Setup time parameterization
  int time_parameterization_task = tf->addNode(std::make_unique<IterativeSplineParameterizationTaskGenerator>(), true);

  // Setup trajectory smoothing
  int smoothing_task{ std::numeric_limits<int>::min() };
  if (post_smoothing)
    smoothing_task = tf->addNode(std::make_unique<RuckigTrajectorySmoothingTaskGenerator>(), true);

  if (check_input)
    tf->addEdges(check_input_task, { GraphTaskflow::ERROR_NODE, has_seed_task });

  tf->addEdges(has_seed_task, { interpolator_task, seed_min_length_task });
  tf->addEdges(interpolator_task, { GraphTaskflow::ERROR_NODE, seed_min_length_task });
  tf->addEdges(seed_min_length_task, { motion_planner_task });

  if (post_collision_check)
  {
    tf->addEdges(motion_planner_task, { GraphTaskflow::ERROR_NODE, contact_check_task });
    tf->addEdges(contact_check_task, { GraphTaskflow::ERROR_NODE, time_parameterization_task });
  }
  else
  {
    tf->addEdges(motion_planner_task, { GraphTaskflow::ERROR_NODE, time_parameterization_task });
  }

  if (post_smoothing)
  {
    tf->addEdges(time_parameterization_task, { GraphTaskflow::ERROR_NODE, smoothing_task });
    tf->addEdges(smoothing_task, { GraphTaskflow::DONE_NODE });
  }
  else
  {
    tf->addEdges(time_parameterization_task, { GraphTaskflow::ERROR_NODE, GraphTaskflow::DONE_NODE });
  }

  return tf;
}

TaskflowGenerator::UPtr createDescartesGenerator(bool check_input, bool post_collision_check, bool post_smoothing)
{
  auto tf = std::make_unique<GraphTaskflow>("DescartesTaskflow");

  int check_input_task{ std::numeric_limits<int>::min() };
  if (check_input)
    check_input_task = tf->addNode(std::make_unique<CheckInputTaskGenerator>(), true);

  // Check if seed was provided
  int has_seed_task = tf->addNode(std::make_unique<HasSeedTaskGenerator>(), true);

  // Simple planner as interpolator
  auto interpolator = std::make_shared<SimpleMotionPlanner>();
  int interpolator_task = tf->addNode(std::make_unique<MotionPlannerTaskGenerator>(interpolator), true);

  // Setup Seed Min Length Process Generator
  // This is required because trajopt requires a minimum length trajectory. This is used to correct the seed if it is
  // to short.
  int seed_min_length_task = tf->addNode(std::make_unique<SeedMinLengthTaskGenerator>());

  // Setup Descartes
  auto motion_planner = std::make_shared<DescartesMotionPlannerF>();
  int motion_planner_task = tf->addNode(std::make_unique<MotionPlannerTaskGenerator>(motion_planner), true);

  // Setup post collision check
  int contact_check_task{ std::numeric_limits<int>::min() };
  if (post_collision_check)
    contact_check_task = tf->addNode(std::make_unique<DiscreteContactCheckTaskGenerator>(), true);

  // Setup time parameterization
  int time_parameterization_task = tf->addNode(std::make_unique<IterativeSplineParameterizationTaskGenerator>(), true);

  // Setup trajectory smoothing
  int smoothing_task{ std::numeric_limits<int>::min() };
  if (post_smoothing)
    smoothing_task = tf->addNode(std::make_unique<RuckigTrajectorySmoothingTaskGenerator>(), true);

  if (check_input)
    tf->addEdges(check_input_task, { GraphTaskflow::ERROR_NODE, has_seed_task });

  tf->addEdges(has_seed_task, { interpolator_task, seed_min_length_task });
  tf->addEdges(interpolator_task, { GraphTaskflow::ERROR_NODE, seed_min_length_task });
  tf->addEdges(seed_min_length_task, { motion_planner_task });

  if (post_collision_check)
  {
    tf->addEdges(motion_planner_task, { GraphTaskflow::ERROR_NODE, contact_check_task });
    tf->addEdges(contact_check_task, { GraphTaskflow::ERROR_NODE, time_parameterization_task });
  }
  else
  {
    tf->addEdges(motion_planner_task, { GraphTaskflow::ERROR_NODE, time_parameterization_task });
  }

  if (post_smoothing)
  {
    tf->addEdges(time_parameterization_task, { GraphTaskflow::ERROR_NODE, smoothing_task });
    tf->addEdges(smoothing_task, { GraphTaskflow::DONE_NODE });
  }
  else
  {
    tf->addEdges(time_parameterization_task, { GraphTaskflow::ERROR_NODE, GraphTaskflow::DONE_NODE });
  }

  return tf;
}

TaskflowGenerator::UPtr createDescartesOnlyGenerator(bool check_input)
{
  auto tf = std::make_unique<GraphTaskflow>("DescartesTaskflow");

  int check_input_task{ std::numeric_limits<int>::min() };
  if (check_input)
    check_input_task = tf->addNode(std::make_unique<CheckInputTaskGenerator>(), true);

  // Check if seed was provided
  int has_seed_task = tf->addNode(std::make_unique<HasSeedTaskGenerator>(), true);

  // Simple planner as interpolator
  auto interpolator = std::make_shared<SimpleMotionPlanner>();
  int interpolator_task = tf->addNode(std::make_unique<MotionPlannerTaskGenerator>(interpolator), true);

  // Setup Seed Min Length Process Generator
  // This is required because trajopt requires a minimum length trajectory. This is used to correct the seed if it is
  // to short.
  int seed_min_length_task = tf->addNode(std::make_unique<SeedMinLengthTaskGenerator>());

  // Setup Descartes
  auto motion_planner = std::make_shared<DescartesMotionPlannerF>();
  int motion_planner_task = tf->addNode(std::make_unique<MotionPlannerTaskGenerator>(motion_planner), true);

  if (check_input)
    tf->addEdges(check_input_task, { GraphTaskflow::ERROR_NODE, has_seed_task });

  tf->addEdges(has_seed_task, { interpolator_task, seed_min_length_task });
  tf->addEdges(interpolator_task, { GraphTaskflow::ERROR_NODE, seed_min_length_task });
  tf->addEdges(seed_min_length_task, { motion_planner_task });
  tf->addEdges(motion_planner_task, { GraphTaskflow::ERROR_NODE, GraphTaskflow::DONE_NODE });

  return tf;
}

TaskflowGenerator::UPtr createCartesianGenerator(bool check_input)
{
  auto tf = std::make_unique<GraphTaskflow>("CartesianTaskflow");

  int check_input_task{ std::numeric_limits<int>::min() };
  if (check_input)
    check_input_task = tf->addNode(std::make_unique<CheckInputTaskGenerator>(), true);

  // Check if seed was provided
  int has_seed_task = tf->addNode(std::make_unique<HasSeedTaskGenerator>(), true);

  // Simple planner as interpolator
  auto interpolator = std::make_shared<SimpleMotionPlanner>();
  int interpolator_task = tf->addNode(std::make_unique<MotionPlannerTaskGenerator>(interpolator), true);

  // Setup Seed Min Length Process Generator
  // This is required because trajopt requires a minimum length trajectory. This is used to correct the seed if it is
  // to short.
  int seed_min_length_task = tf->addNode(std::make_unique<SeedMinLengthTaskGenerator>());

  // Setup Descartes
  auto descartes_planner = std::make_shared<DescartesMotionPlannerF>();
  int descartes_planner_task = tf->addNode(std::make_unique<MotionPlannerTaskGenerator>(descartes_planner), true);

  // Setup TrajOpt
  auto trajopt_planner = std::make_shared<TrajOptMotionPlanner>();
  int trajopt_planner_task = tf->addNode(std::make_unique<MotionPlannerTaskGenerator>(trajopt_planner), true);

  // Setup post collision check
  int contact_check_task = tf->addNode(std::make_unique<DiscreteContactCheckTaskGenerator>(), true);

  // Setup time parameterization
  int time_parameterization_task = tf->addNode(std::make_unique<IterativeSplineParameterizationTaskGenerator>(), true);

  if (check_input)
    tf->addEdges(check_input_task, { GraphTaskflow::ERROR_NODE, has_seed_task });

  tf->addEdges(has_seed_task, { interpolator_task, seed_min_length_task });
  tf->addEdges(interpolator_task, { GraphTaskflow::ERROR_NODE, seed_min_length_task });
  tf->addEdges(seed_min_length_task, { descartes_planner_task });
  tf->addEdges(descartes_planner_task, { GraphTaskflow::ERROR_NODE, trajopt_planner_task });
  tf->addEdges(trajopt_planner_task, { GraphTaskflow::ERROR_NODE, contact_check_task });
  tf->addEdges(contact_check_task, { GraphTaskflow::ERROR_NODE, time_parameterization_task });
  tf->addEdges(time_parameterization_task, { GraphTaskflow::ERROR_NODE, GraphTaskflow::DONE_NODE });

  return tf;
}

TaskflowGenerator::UPtr createFreespaceGenerator(bool check_input)
{
  auto tf = std::make_unique<GraphTaskflow>("FreespaceTaskflow");

  int check_input_task{ std::numeric_limits<int>::min() };
  if (check_input)
    check_input_task = tf->addNode(std::make_unique<CheckInputTaskGenerator>(), true);

  // Check if seed was provided
  int has_seed_task = tf->addNode(std::make_unique<HasSeedTaskGenerator>(), true);

  // Simple planner as interpolator
  auto interpolator = std::make_shared<SimpleMotionPlanner>();
  int interpolator_task = tf->addNode(std::make_unique<MotionPlannerTaskGenerator>(interpolator), true);

  // Setup Seed Min Length Process Generator
  // This is required because trajopt requires a minimum length trajectory. This is used to correct the seed if it is
  // to short.
  int seed_min_length_task = tf->addNode(std::make_unique<SeedMinLengthTaskGenerator>());

  // Setup OMPL
  auto ompl_planner = std::make_shared<OMPLMotionPlanner>();
  int ompl_planner_task = tf->addNode(std::make_unique<MotionPlannerTaskGenerator>(ompl_planner), true);

  // Setup TrajOpt
  auto trajopt_planner = std::make_shared<TrajOptMotionPlanner>();
  int trajopt_planner_task = tf->addNode(std::make_unique<MotionPlannerTaskGenerator>(trajopt_planner), true);

  // Setup post collision check
  int contact_check_task = tf->addNode(std::make_unique<DiscreteContactCheckTaskGenerator>(), true);

  // Setup time parameterization
  int time_parameterization_task = tf->addNode(std::make_unique<IterativeSplineParameterizationTaskGenerator>(), true);

  if (check_input)
    tf->addEdges(check_input_task, { GraphTaskflow::ERROR_NODE, has_seed_task });

  tf->addEdges(has_seed_task, { interpolator_task, seed_min_length_task });
  tf->addEdges(interpolator_task, { GraphTaskflow::ERROR_NODE, seed_min_length_task });
  tf->addEdges(seed_min_length_task, { ompl_planner_task });
  tf->addEdges(ompl_planner_task, { GraphTaskflow::ERROR_NODE, trajopt_planner_task });
  tf->addEdges(trajopt_planner_task, { GraphTaskflow::ERROR_NODE, contact_check_task });
  tf->addEdges(contact_check_task, { GraphTaskflow::ERROR_NODE, time_parameterization_task });
  tf->addEdges(time_parameterization_task, { GraphTaskflow::ERROR_NODE, GraphTaskflow::DONE_NODE });

  return tf;
}

TaskflowGenerator::UPtr createFreespaceTrajOptFirstGenerator(bool check_input)
{
  auto tf = std::make_unique<GraphTaskflow>("FreespaceTrajOptFirstTaskflow");

  int check_input_task{ std::numeric_limits<int>::min() };
  if (check_input)
    check_input_task = tf->addNode(std::make_unique<CheckInputTaskGenerator>(), true);

  // Check if seed was provided
  int has_seed_task = tf->addNode(std::make_unique<HasSeedTaskGenerator>(), true);

  // Simple planner as interpolator
  auto interpolator = std::make_shared<SimpleMotionPlanner>();
  int interpolator_task = tf->addNode(std::make_unique<MotionPlannerTaskGenerator>(interpolator), true);

  // Setup Seed Min Length Process Generator
  // This is required because trajopt requires a minimum length trajectory. This is used to correct the seed if it is
  // to short.
  int seed_min_length_task = tf->addNode(std::make_unique<SeedMinLengthTaskGenerator>());

  // Setup TrajOpt
  auto trajopt_planner1 = std::make_shared<TrajOptMotionPlanner>();
  int trajopt_planner1_task = tf->addNode(std::make_unique<MotionPlannerTaskGenerator>(trajopt_planner1), true);

  // Setup OMPL
  auto ompl_planner = std::make_shared<OMPLMotionPlanner>();
  int ompl_planner_task = tf->addNode(std::make_unique<MotionPlannerTaskGenerator>(ompl_planner), true);

  // Setup TrajOpt
  auto trajopt_planner2 = std::make_shared<TrajOptMotionPlanner>();
  int trajopt_planner2_task = tf->addNode(std::make_unique<MotionPlannerTaskGenerator>(trajopt_planner2), true);

  // Setup post collision check
  int contact_check_task = tf->addNode(std::make_unique<DiscreteContactCheckTaskGenerator>(), true);

  // Setup time parameterization
  int time_parameterization_task = tf->addNode(std::make_unique<IterativeSplineParameterizationTaskGenerator>(), true);

  if (check_input)
    tf->addEdges(check_input_task, { GraphTaskflow::ERROR_NODE, has_seed_task });

  tf->addEdges(has_seed_task, { interpolator_task, seed_min_length_task });
  tf->addEdges(interpolator_task, { GraphTaskflow::ERROR_NODE, seed_min_length_task });
  tf->addEdges(seed_min_length_task, { trajopt_planner1_task });
  tf->addEdges(trajopt_planner1_task, { ompl_planner_task, contact_check_task });
  tf->addEdges(ompl_planner_task, { GraphTaskflow::ERROR_NODE, trajopt_planner2_task });
  tf->addEdges(trajopt_planner2_task, { GraphTaskflow::ERROR_NODE, contact_check_task });
  tf->addEdges(contact_check_task, { GraphTaskflow::ERROR_NODE, time_parameterization_task });
  tf->addEdges(time_parameterization_task, { GraphTaskflow::ERROR_NODE, GraphTaskflow::DONE_NODE });

  return tf;
}

TaskflowGenerator::UPtr createRasterGenerator()
{
  // Create Freespace and Transition Taskflows
  TaskflowGenerator::UPtr freespace_task = createFreespaceGenerator(false);
  TaskflowGenerator::UPtr transition_task = createFreespaceGenerator(false);

  // Create Raster Taskflow
  TaskflowGenerator::UPtr raster_task = createCartesianGenerator(false);

  return std::make_unique<RasterTaskflow>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

TaskflowGenerator::UPtr createRasterOnlyGenerator()
{
  // Create Freespace and Transition Taskflows
  TaskflowGenerator::UPtr transition_task = createFreespaceGenerator(false);

  // Create Raster Taskflow
  TaskflowGenerator::UPtr raster_task = createCartesianGenerator(false);

  return std::make_unique<RasterOnlyTaskflow>(std::move(transition_task), std::move(raster_task));
}

TaskflowGenerator::UPtr createRasterGlobalGenerator()
{
  TaskflowGenerator::UPtr global_task = createDescartesOnlyGenerator(false);
  TaskflowGenerator::UPtr freespace_task = createFreespaceTrajOptFirstGenerator(false);
  TaskflowGenerator::UPtr transition_task = createFreespaceTrajOptFirstGenerator(false);
  TaskflowGenerator::UPtr raster_task = createTrajOptGenerator(false);

  return std::make_unique<RasterGlobalTaskflow>(
      std::move(global_task), std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

TaskflowGenerator::UPtr createRasterDTGenerator()
{
  // Create Freespace and Transition Taskflows
  TaskflowGenerator::UPtr freespace_task = createFreespaceGenerator(false);
  TaskflowGenerator::UPtr transition_task = createFreespaceGenerator(false);

  // Create Raster Taskflow
  TaskflowGenerator::UPtr raster_task = createCartesianGenerator(false);

  return std::make_unique<RasterDTTaskflow>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

TaskflowGenerator::UPtr createRasterWAADGenerator()
{
  // Create Freespace and Transition Taskflows
  TaskflowGenerator::UPtr freespace_task = createFreespaceGenerator(false);
  TaskflowGenerator::UPtr transition_task = createFreespaceGenerator(false);

  // Create Raster Taskflow
  TaskflowGenerator::UPtr raster_task = createCartesianGenerator();

  return std::make_unique<RasterWAADTaskflow>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

TaskflowGenerator::UPtr createRasterWAADDTGenerator()
{
  // Create Freespace and Transition Taskflows
  TaskflowGenerator::UPtr freespace_task = createFreespaceGenerator(false);
  TaskflowGenerator::UPtr transition_task = createFreespaceGenerator(false);

  // Create Raster Taskflow
  TaskflowGenerator::UPtr raster_task = createCartesianGenerator(false);

  return std::make_unique<RasterWAADDTTaskflow>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

TaskflowGenerator::UPtr createRasterOnlyGlobalGenerator()
{
  TaskflowGenerator::UPtr global_task = createDescartesOnlyGenerator(false);
  TaskflowGenerator::UPtr transition_task = createFreespaceTrajOptFirstGenerator(false);
  TaskflowGenerator::UPtr raster_task = createTrajOptGenerator(false);

  return std::make_unique<RasterOnlyGlobalTaskflow>(
      std::move(global_task), std::move(transition_task), std::move(raster_task));
}

TaskflowGenerator::UPtr createRasterCTGenerator()
{
  // Create Freespace and Transition Taskflows
  TaskflowGenerator::UPtr freespace_task = createFreespaceGenerator(false);

  // Create Raster Taskflow
  TaskflowGenerator::UPtr raster_task = createCartesianGenerator(false);
  TaskflowGenerator::UPtr transition_task = createCartesianGenerator(false);

  return std::make_unique<RasterTaskflow>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

TaskflowGenerator::UPtr createRasterOnlyCTGenerator()
{
  // Create Transition and Raster Taskflow
  TaskflowGenerator::UPtr raster_task = createCartesianGenerator(false);
  TaskflowGenerator::UPtr transition_task = createCartesianGenerator(false);

  return std::make_unique<RasterOnlyTaskflow>(std::move(transition_task), std::move(raster_task));
}

TaskflowGenerator::UPtr createRasterCTDTGenerator()
{
  // Create Freespace and Transition Taskflows
  TaskflowGenerator::UPtr freespace_task = createFreespaceGenerator(false);

  // Create Raster Taskflow
  TaskflowGenerator::UPtr raster_task = createCartesianGenerator(false);
  TaskflowGenerator::UPtr transition_task = createCartesianGenerator(false);

  return std::make_unique<RasterDTTaskflow>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

TaskflowGenerator::UPtr createRasterCTWAADGenerator()
{
  // Create Freespace and Transition Taskflows
  TaskflowGenerator::UPtr freespace_task = createFreespaceGenerator(false);

  // Create Raster Taskflow
  TaskflowGenerator::UPtr raster_task = createCartesianGenerator(false);
  TaskflowGenerator::UPtr transition_task = createCartesianGenerator(false);

  return std::make_unique<RasterWAADTaskflow>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

TaskflowGenerator::UPtr createRasterCTWAADDTGenerator()
{
  // Create Freespace and Transition Taskflows
  TaskflowGenerator::UPtr freespace_task = createFreespaceGenerator(false);

  // Create Raster Taskflow
  TaskflowGenerator::UPtr raster_task = createCartesianGenerator(false);
  TaskflowGenerator::UPtr transition_task = createCartesianGenerator(false);

  return std::make_unique<RasterWAADDTTaskflow>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

TaskflowGenerator::UPtr createRasterGlobalCTGenerator()
{
  TaskflowGenerator::UPtr global_task = createDescartesOnlyGenerator(false);
  TaskflowGenerator::UPtr freespace_task = createFreespaceTrajOptFirstGenerator(false);
  TaskflowGenerator::UPtr raster_task = createTrajOptGenerator(false);
  TaskflowGenerator::UPtr transition_task = createTrajOptGenerator(false);

  return std::make_unique<RasterGlobalTaskflow>(
      std::move(global_task), std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

TaskflowGenerator::UPtr createRasterOnlyGlobalCTGenerator()
{
  TaskflowGenerator::UPtr global_task = createDescartesOnlyGenerator(false);
  TaskflowGenerator::UPtr raster_task = createTrajOptGenerator(false);
  TaskflowGenerator::UPtr transition_task = createTrajOptGenerator(false);

  return std::make_unique<RasterOnlyGlobalTaskflow>(
      std::move(global_task), std::move(transition_task), std::move(raster_task));
}
}  // namespace tesseract_planning
