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
#include <yaml-cpp/yaml.h>

#include <tesseract_environment/environment.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_task.h>

#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>

#include <tesseract_motion_planners/core/planner.h>
#include <tesseract_motion_planners/core/types.h>

namespace tesseract_planning
{
class TaskComposerPluginFactory;

template <typename MotionPlannerType>
class MotionPlannerTask : public TaskComposerTask
{
public:
  // Requried
  static const std::string INOUT_PROGRAM_PORT;
  static const std::string INPUT_ENVIRONMENT_PORT;
  static const std::string INPUT_PROFILES_PORT;

  // Optional
  static const std::string INPUT_MANIP_INFO_PORT;

  MotionPlannerTask() : TaskComposerTask("MotionPlannerTask", MotionPlannerTask<MotionPlannerType>::ports(), true) {}
  explicit MotionPlannerTask(std::string name,  // NOLINT(performance-unnecessary-value-param)
                             std::string input_program_key,
                             std::string input_environment_key,
                             std::string input_profiles_key,
                             std::string output_program_key,
                             bool format_result_as_input,
                             bool conditional)
    : TaskComposerTask(std::move(name), MotionPlannerTask<MotionPlannerType>::ports(), conditional)
    , planner_(std::make_shared<MotionPlannerType>(ns_))
    , format_result_as_input_(format_result_as_input)
  {
    input_keys_.add(INOUT_PROGRAM_PORT, std::move(input_program_key));
    input_keys_.add(INPUT_ENVIRONMENT_PORT, std::move(input_environment_key));
    input_keys_.add(INPUT_PROFILES_PORT, std::move(input_profiles_key));
    output_keys_.add(INOUT_PROGRAM_PORT, std::move(output_program_key));
    validatePorts();
  }

  explicit MotionPlannerTask(std::string name,  // NOLINT(performance-unnecessary-value-param)
                             const YAML::Node& config,
                             const TaskComposerPluginFactory& /*plugin_factory*/)
    : TaskComposerTask(std::move(name), MotionPlannerTask<MotionPlannerType>::ports(), config)
    , planner_(std::make_shared<MotionPlannerType>(ns_))
  {
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

  static TaskComposerNodePorts ports()
  {
    TaskComposerNodePorts ports;
    ports.input_required[INOUT_PROGRAM_PORT] = TaskComposerNodePorts::SINGLE;
    ports.input_required[INPUT_ENVIRONMENT_PORT] = TaskComposerNodePorts::SINGLE;
    ports.input_required[INPUT_PROFILES_PORT] = TaskComposerNodePorts::SINGLE;

    ports.input_optional[INPUT_MANIP_INFO_PORT] = TaskComposerNodePorts::SINGLE;

    ports.output_required[INOUT_PROGRAM_PORT] = TaskComposerNodePorts::SINGLE;
    return ports;
  }

  std::unique_ptr<TaskComposerNodeInfo> runImpl(TaskComposerContext& context,
                                                OptionalTaskComposerExecutor /*executor*/ = std::nullopt) const override
  {
    auto info = std::make_unique<TaskComposerNodeInfo>(*this);
    info->return_value = 0;
    info->status_code = 0;

    // --------------------
    // Check that inputs are valid
    // --------------------
    auto env_poly = getData(*context.data_storage, INPUT_ENVIRONMENT_PORT);
    if (env_poly.getType() != std::type_index(typeid(std::shared_ptr<const tesseract_environment::Environment>)))
    {
      info->status_code = 0;
      info->status_message = "Input data '" + input_keys_.get(INPUT_ENVIRONMENT_PORT) + "' is not correct type";
      CONSOLE_BRIDGE_logError("%s", info->status_message.c_str());
      info->return_value = 0;
      return info;
    }

    auto env = env_poly.template as<std::shared_ptr<const tesseract_environment::Environment>>();

    auto input_data_poly = getData(*context.data_storage, INOUT_PROGRAM_PORT);
    if (input_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
    {
      info->status_message = "Input instructions to MotionPlannerTask: " + name_ + " must be a composite instruction";
      CONSOLE_BRIDGE_logError("%s", info->status_message.c_str());
      return info;
    }
    tesseract_common::AnyPoly original_input_data_poly{ input_data_poly };

    auto profiles =
        getData(*context.data_storage, INPUT_PROFILES_PORT).template as<std::shared_ptr<ProfileDictionary>>();

    tesseract_common::ManipulatorInfo input_manip_info;
    auto manip_info_poly = getData(*context.data_storage, INPUT_MANIP_INFO_PORT, false);
    if (!manip_info_poly.isNull())
      input_manip_info = manip_info_poly.template as<tesseract_common::ManipulatorInfo>();

    // Make a non-const copy of the input instructions to update the start/end
    auto& instructions = input_data_poly.template as<CompositeInstruction>();
    assert(!(input_manip_info.empty() && instructions.getManipulatorInfo().empty()));
    instructions.setManipulatorInfo(instructions.getManipulatorInfo().getCombined(input_manip_info));

    // --------------------
    // Fill out request
    // --------------------
    PlannerRequest request;
    request.env = env;
    request.instructions = instructions;
    request.profiles = profiles;
    request.format_result_as_input = format_result_as_input_;

    // --------------------
    // Fill out response
    // --------------------
    request.verbose = false;
    if (console_bridge::getLogLevel() == console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG)
      request.verbose = true;
    PlannerResponse response = planner_->solve(request);
    setData(*context.data_storage, INOUT_PROGRAM_PORT, response.results);

    // --------------------
    // Verify Success
    // --------------------
    if (response)
    {
      info->return_value = 1;
      info->color = "green";
      info->status_code = 1;
      info->status_message = response.message;
      CONSOLE_BRIDGE_logDebug("Motion Planner process succeeded");
      return info;
    }

    CONSOLE_BRIDGE_logInform("%s motion planning failed (%s) for process input: %s",
                             planner_->getName().c_str(),
                             response.message.c_str(),
                             instructions.getDescription().c_str());

    // If the output key is not the same as the input key the output data should be assigned the input data for error
    // branching
    if (output_keys_.get(INOUT_PROGRAM_PORT) != input_keys_.get(INOUT_PROGRAM_PORT))
      setData(*context.data_storage, INOUT_PROGRAM_PORT, original_input_data_poly);

    info->status_message = response.message;
    return info;
  }
};

// Requried
template <typename MotionPlannerType>
const std::string MotionPlannerTask<MotionPlannerType>::INOUT_PROGRAM_PORT = "program";

template <typename MotionPlannerType>
const std::string MotionPlannerTask<MotionPlannerType>::INPUT_ENVIRONMENT_PORT = "environment";

template <typename MotionPlannerType>
const std::string MotionPlannerTask<MotionPlannerType>::INPUT_PROFILES_PORT = "profiles";

// Optional
template <typename MotionPlannerType>
const std::string MotionPlannerTask<MotionPlannerType>::INPUT_MANIP_INFO_PORT = "manip_info";

}  // namespace tesseract_planning

#endif  // TESSERACT_TASK_COMPOSER_MOTION_PLANNER_TASK_HPP
