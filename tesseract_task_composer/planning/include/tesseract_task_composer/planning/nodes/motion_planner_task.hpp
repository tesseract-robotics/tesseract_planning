/**
 * @file motion_planner_task.h
 * @brief Task Composer motion planner task
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
#ifndef TESSERACT_TASK_COMPOSER_MOTION_PLANNER_TASK_HPP
#define TESSERACT_TASK_COMPOSER_MOTION_PLANNER_TASK_HPP

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <boost/serialization/access.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_task.h>
#include <tesseract_task_composer/planning/nodes/motion_planner_task_info.h>
#include <tesseract_task_composer/planning/planning_task_composer_problem.h>
#include <tesseract_motion_planners/core/planner.h>

namespace tesseract_planning
{
class TaskComposerPluginFactory;

template <typename MotionPlannerType>
class MotionPlannerTask : public TaskComposerTask
{
public:
  MotionPlannerTask() : TaskComposerTask("MotionPlannerTask", true) {}
  explicit MotionPlannerTask(std::string name,  // NOLINT(performance-unnecessary-value-param)
                             std::string input_key,
                             std::string output_key,
                             bool format_result_as_input,
                             bool conditional)
    : TaskComposerTask(std::move(name), conditional)
    , planner_(std::make_shared<MotionPlannerType>(name_))
    , format_result_as_input_(format_result_as_input)
  {
    input_keys_.push_back(std::move(input_key));
    output_keys_.push_back(std::move(output_key));
  }

  explicit MotionPlannerTask(std::string name,  // NOLINT(performance-unnecessary-value-param)
                             const YAML::Node& config,
                             const TaskComposerPluginFactory& /*plugin_factory*/)
    : TaskComposerTask(std::move(name), config), planner_(std::make_shared<MotionPlannerType>(name_))
  {
    if (input_keys_.empty())
      throw std::runtime_error("MotionPlannerTask, config missing 'inputs' entry");

    if (input_keys_.size() > 1)
      throw std::runtime_error("MotionPlannerTask, config 'inputs' entry currently only supports one input key");

    if (output_keys_.empty())
      throw std::runtime_error("MotionPlannerTask, config missing 'outputs' entry");

    if (output_keys_.size() > 1)
      throw std::runtime_error("MotionPlannerTask, config 'outputs' entry currently only supports one output key");

    try
    {
      if (YAML::Node n = config["format_result_as_input"])
        format_result_as_input_ = n.as<bool>();
    }
    catch (const std::exception& e)
    {
      throw std::runtime_error("MotionPlannerTask: Failed to parse yaml config data! Details: " +
                               std::string(e.what()));
    }
  }
  ~MotionPlannerTask() override = default;

  bool operator==(const MotionPlannerTask& rhs) const
  {
    bool equal = true;
    equal &= (format_result_as_input_ == rhs.format_result_as_input_);
    equal &= TaskComposerTask::operator==(rhs);
    return equal;
  }

  bool operator!=(const MotionPlannerTask& rhs) const { return !operator==(rhs); }

protected:
  std::shared_ptr<MotionPlannerType> planner_;
  bool format_result_as_input_{ true };

  friend struct tesseract_common::Serialization;
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
  {
    ar& BOOST_SERIALIZATION_NVP(format_result_as_input_);
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
  }

  TaskComposerNodeInfo::UPtr runImpl(TaskComposerContext& context,
                                     OptionalTaskComposerExecutor /*executor*/ = std::nullopt) const override
  {
    // Get the problem
    auto& problem = dynamic_cast<PlanningTaskComposerProblem&>(*context.problem);

    auto info = std::make_unique<MotionPlannerTaskInfo>(*this);
    info->return_value = 0;
    info->env = problem.env;

    // --------------------
    // Check that inputs are valid
    // --------------------
    auto input_data_poly = context.data_storage->getData(input_keys_[0]);
    if (input_data_poly.isNull() || input_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
    {
      info->message = "Input instructions to MotionPlannerTask: " + name_ + " must be a composite instruction";
      CONSOLE_BRIDGE_logError("%s", info->message.c_str());
      return info;
    }

    // Make a non-const copy of the input instructions to update the start/end
    auto& instructions = input_data_poly.template as<CompositeInstruction>();
    assert(!(problem.manip_info.empty() && instructions.getManipulatorInfo().empty()));
    instructions.setManipulatorInfo(instructions.getManipulatorInfo().getCombined(problem.manip_info));

    // --------------------
    // Fill out request
    // --------------------
    PlannerRequest request;
    request.env_state = problem.env->getState();
    request.env = problem.env;
    request.instructions = instructions;
    request.profiles = problem.profiles;
    request.plan_profile_remapping = problem.move_profile_remapping;
    request.composite_profile_remapping = problem.composite_profile_remapping;
    request.format_result_as_input = format_result_as_input_;

    // --------------------
    // Fill out response
    // --------------------
    request.verbose = false;
    if (console_bridge::getLogLevel() == console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG)
      request.verbose = true;
    PlannerResponse response = planner_->solve(request);

    // --------------------
    // Verify Success
    // --------------------
    if (response)
    {
      context.data_storage->setData(output_keys_[0], response.results);

      info->return_value = 1;
      info->color = "green";
      info->message = response.message;
      CONSOLE_BRIDGE_logDebug("Motion Planner process succeeded");
      return info;
    }

    CONSOLE_BRIDGE_logInform("%s motion planning failed (%s) for process input: %s",
                             planner_->getName().c_str(),
                             response.message.c_str(),
                             instructions.getDescription().c_str());

    // If the output key is not the same as the input key the output data should be assigned the input data for error
    // branching
    if (output_keys_[0] != input_keys_[0])
      context.data_storage->setData(output_keys_[0], context.data_storage->getData(input_keys_[0]));

    info->message = response.message;
    return info;
  }
};

}  // namespace tesseract_planning

#endif  // TESSERACT_TASK_COMPOSER_MOTION_PLANNER_TASK_HPP
