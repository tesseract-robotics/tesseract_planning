/**
 * @file seed_length_task.h
 * @brief Task for processing the seed so it meets a minimum length. Planners like trajopt need
 * at least 10 states in the trajectory to perform velocity, acceleration and jerk smoothing.
 *
 * @author Levi Armstrong
 * @date November 2. 2020
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
#ifndef TESSERACT_TASK_COMPOSER_SEED_MIN_LENGTH_TASK_H
#define TESSERACT_TASK_COMPOSER_SEED_MIN_LENGTH_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/task_composer_task.h>
#include <tesseract_task_composer/task_composer_node_info.h>
#include <tesseract_task_composer/nodes/default_task_namespaces.h>
#include <tesseract_command_language/composite_instruction.h>

namespace tesseract_planning
{
class SeedMinLengthTask : public TaskComposerTask
{
public:
  using Ptr = std::shared_ptr<SeedMinLengthTask>;
  using ConstPtr = std::shared_ptr<const SeedMinLengthTask>;
  using UPtr = std::unique_ptr<SeedMinLengthTask>;
  using ConstUPtr = std::unique_ptr<const SeedMinLengthTask>;

  SeedMinLengthTask() = default;  // Required for serialization
  SeedMinLengthTask(std::string input_key,
                    std::string output_key,
                    bool is_conditional = false,
                    std::string name = profile_ns::SEED_MIN_LENGTH_DEFAULT_NAMESPACE);
  ~SeedMinLengthTask() override = default;
  SeedMinLengthTask(const SeedMinLengthTask&) = delete;
  SeedMinLengthTask& operator=(const SeedMinLengthTask&) = delete;
  SeedMinLengthTask(SeedMinLengthTask&&) = delete;
  SeedMinLengthTask& operator=(SeedMinLengthTask&&) = delete;

  int run(TaskComposerInput& input) const override final;

  bool operator==(const SeedMinLengthTask& rhs) const;
  bool operator!=(const SeedMinLengthTask& rhs) const;

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  void subdivide(CompositeInstruction& composite,
                 const CompositeInstruction& current_composite,
                 InstructionPoly& start_instruction,
                 int subdivisions) const;

  std::string input_key_;
  std::string output_key_;
};

class SeedMinLengthTaskInfo : public TaskComposerNodeInfo
{
public:
  using Ptr = std::shared_ptr<SeedMinLengthTaskInfo>;
  using ConstPtr = std::shared_ptr<const SeedMinLengthTaskInfo>;
  using UPtr = std::unique_ptr<SeedMinLengthTaskInfo>;
  using ConstUPtr = std::unique_ptr<const SeedMinLengthTaskInfo>;

  SeedMinLengthTaskInfo() = default;
  SeedMinLengthTaskInfo(boost::uuids::uuid uuid, std::string name = profile_ns::SEED_MIN_LENGTH_DEFAULT_NAMESPACE);

  TaskComposerNodeInfo::UPtr clone() const override;

  bool operator==(const SeedMinLengthTaskInfo& rhs) const;
  bool operator!=(const SeedMinLengthTaskInfo& rhs) const;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::SeedMinLengthTask, "SeedMinLengthTask")
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::SeedMinLengthTaskInfo, "SeedMinLengthTaskInfo")
#endif  // TESSERACT_TASK_COMPOSER_SEED_MIN_LENGTH_TASK_H
