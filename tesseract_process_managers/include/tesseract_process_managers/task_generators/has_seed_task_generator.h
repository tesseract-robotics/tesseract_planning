/**
 * @file has_seed_task_generator.h
 * @brief Process generator for checking if the request already has a seed
 *
 * @author Levi Armstrong
 * @date November 2. 2021
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
#ifndef TESSERACT_PROCESS_MANAGERS_HAS_SEED_TASK_GENERATOR_H
#define TESSERACT_PROCESS_MANAGERS_HAS_SEED_TASK_GENERATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/task_generator.h>
#include <tesseract_process_managers/core/default_task_namespaces.h>

namespace tesseract_planning
{
class HasSeedTaskGenerator : public TaskGenerator
{
public:
  using UPtr = std::unique_ptr<HasSeedTaskGenerator>;

  HasSeedTaskGenerator(std::string name = profile_ns::HAS_SEED_DEFAULT_NAMESPACE);

  ~HasSeedTaskGenerator() override = default;
  HasSeedTaskGenerator(const HasSeedTaskGenerator&) = delete;
  HasSeedTaskGenerator& operator=(const HasSeedTaskGenerator&) = delete;
  HasSeedTaskGenerator(HasSeedTaskGenerator&&) = delete;
  HasSeedTaskGenerator& operator=(HasSeedTaskGenerator&&) = delete;

  int conditionalProcess(TaskInput input, std::size_t unique_id) const override final;

  void process(TaskInput input, std::size_t unique_id) const override final;
};

class HasSeedTaskInfo : public TaskInfo
{
public:
  using Ptr = std::shared_ptr<HasSeedTaskInfo>;
  using ConstPtr = std::shared_ptr<const HasSeedTaskInfo>;

  HasSeedTaskInfo() = default;
  HasSeedTaskInfo(std::size_t unique_id, std::string name = profile_ns::HAS_SEED_DEFAULT_NAMESPACE);

  TaskInfo::UPtr clone() const override;

  bool operator==(const HasSeedTaskInfo& rhs) const;
  bool operator!=(const HasSeedTaskInfo& rhs) const;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::HasSeedTaskInfo, "HasSeedTaskInfo")
#endif  // TESSERACT_PROCESS_MANAGERS_HAS_SEED_TASK_GENERATOR_H
