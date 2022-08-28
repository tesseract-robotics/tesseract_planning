/**
 * @file cartesian_motion_planner_task.h
 * @brief Cartesian motion planning pipeline
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <boost/serialization/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_common/timer.h>

#include <tesseract_task_composer/nodes/cartesian_motion_pipeline_task.h>

#include <tesseract_task_composer/nodes/motion_planner_task.h>
#include <tesseract_task_composer/nodes/has_seed_task.h>
#include <tesseract_task_composer/nodes/min_length_task.h>
#include <tesseract_task_composer/nodes/discrete_contact_check_task.h>
#include <tesseract_task_composer/nodes/iterative_spline_parameterization_task.h>
#include <tesseract_task_composer/nodes/ruckig_trajectory_smoothing_task.h>
#include <tesseract_task_composer/nodes/check_input_task.h>
#include <tesseract_task_composer/nodes/done_task.h>
#include <tesseract_task_composer/nodes/error_task.h>

#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>

namespace tesseract_planning
{
CartesianMotionPipelineTask::CartesianMotionPipelineTask(std::string name)
  : TaskComposerGraph(name), input_key_(uuid_str_), output_key_(uuid_str_)
{
  ctor();
}

CartesianMotionPipelineTask::CartesianMotionPipelineTask(std::string input_key,
                                                         std::string output_key,
                                                         std::string name)
  : TaskComposerGraph(name), input_key_(std::move(input_key)), output_key_(std::move(output_key))
{
  ctor();
}
CartesianMotionPipelineTask::CartesianMotionPipelineTask(std::string input_key,
                                                         std::string output_key,
                                                         bool check_input,
                                                         bool post_collision_check,
                                                         bool post_smoothing,
                                                         std::string name)
  : TaskComposerGraph(name)
  , input_key_(std::move(input_key))
  , output_key_(std::move(output_key))
  , check_input_(check_input)
  , post_collision_check_(post_collision_check)
  , post_smoothing_(post_smoothing)
{
  ctor();
}

void CartesianMotionPipelineTask::ctor()
{
  boost::uuids::uuid done_task = addNode(std::make_unique<DoneTask>());
  boost::uuids::uuid error_task = addNode(std::make_unique<ErrorTask>());

  boost::uuids::uuid check_input_task;
  if (check_input_)
    check_input_task = addNode(std::make_unique<CheckInputTask>(input_key_));

  // Check if seed was provided
  boost::uuids::uuid has_seed_task = addNode(std::make_unique<HasSeedTask>());

  // Simple planner as interpolator
  auto interpolator = std::make_shared<SimpleMotionPlanner>();
  boost::uuids::uuid interpolator_task =
      addNode(std::make_unique<MotionPlannerTask>(interpolator, input_key_, output_key_));

  // Setup Seed Min Length Process Generator
  // This is required because trajopt requires a minimum length trajectory. This is used to correct the seed if it is
  // to short.
  boost::uuids::uuid seed_min_length_task = addNode(std::make_unique<MinLengthTask>(output_key_, output_key_));

  // Setup Descartes
  auto descartes_planner = std::make_shared<DescartesMotionPlannerF>();
  boost::uuids::uuid descartes_planner_task =
      addNode(std::make_unique<MotionPlannerTask>(descartes_planner, output_key_, output_key_));

  // Setup TrajOpt
  auto trajopt_planner = std::make_shared<TrajOptMotionPlanner>();
  boost::uuids::uuid trajopt_planner_task =
      addNode(std::make_unique<MotionPlannerTask>(trajopt_planner, output_key_, output_key_, false));

  // Setup post collision check
  boost::uuids::uuid contact_check_task;
  if (post_collision_check_)
    contact_check_task = addNode(std::make_unique<DiscreteContactCheckTask>(output_key_));

  // Setup time parameterization
  boost::uuids::uuid time_parameterization_task =
      addNode(std::make_unique<IterativeSplineParameterizationTask>(output_key_, output_key_));

  // Setup trajectory smoothing
  boost::uuids::uuid smoothing_task;
  if (post_smoothing_)
    smoothing_task = addNode(std::make_unique<RuckigTrajectorySmoothingTask>(output_key_, output_key_));

  if (check_input_)
    addEdges(check_input_task, { error_task, has_seed_task });

  addEdges(has_seed_task, { interpolator_task, seed_min_length_task });
  addEdges(interpolator_task, { error_task, seed_min_length_task });
  addEdges(seed_min_length_task, { descartes_planner_task });

  if (post_collision_check_)
  {
    addEdges(descartes_planner_task, { error_task, trajopt_planner_task });
    addEdges(trajopt_planner_task, { error_task, contact_check_task });
    addEdges(contact_check_task, { error_task, time_parameterization_task });
  }
  else
  {
    addEdges(descartes_planner_task, { error_task, trajopt_planner_task });
    addEdges(trajopt_planner_task, { error_task, time_parameterization_task });
  }

  if (post_smoothing_)
  {
    addEdges(time_parameterization_task, { error_task, smoothing_task });
    addEdges(smoothing_task, { done_task });
  }
  else
  {
    addEdges(time_parameterization_task, { error_task, done_task });
  }
}

TaskComposerNode::UPtr CartesianMotionPipelineTask::clone() const
{
  return std::make_unique<CartesianMotionPipelineTask>(
      input_key_, output_key_, check_input_, post_collision_check_, post_smoothing_, name_);
}

bool CartesianMotionPipelineTask::operator==(const CartesianMotionPipelineTask& rhs) const
{
  bool equal = true;
  equal &= (input_key_ == rhs.input_key_);
  equal &= (output_key_ == rhs.output_key_);
  equal &= (check_input_ == rhs.check_input_);
  equal &= (post_collision_check_ == rhs.post_collision_check_);
  equal &= (post_smoothing_ == rhs.post_smoothing_);
  equal &= TaskComposerGraph::operator==(rhs);
  return equal;
}
bool CartesianMotionPipelineTask::operator!=(const CartesianMotionPipelineTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void CartesianMotionPipelineTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(input_key_);
  ar& BOOST_SERIALIZATION_NVP(output_key_);
  ar& BOOST_SERIALIZATION_NVP(check_input_);
  ar& BOOST_SERIALIZATION_NVP(post_collision_check_);
  ar& BOOST_SERIALIZATION_NVP(post_smoothing_);
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerGraph);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::CartesianMotionPipelineTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::CartesianMotionPipelineTask)
