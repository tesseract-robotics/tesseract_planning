/**
 * @file has_seed_task.h
 * @brief Task for checking if the request already has a seed
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
#ifndef TESSERACT_TASK_COMPOSER_HAS_SEED_TASK_H
#define TESSERACT_TASK_COMPOSER_HAS_SEED_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/task_composer_task.h>
#include <tesseract_task_composer/nodes/default_task_namespaces.h>

namespace tesseract_planning
{
class HasSeedTask : public TaskComposerTask
{
public:
  using Ptr = std::shared_ptr<HasSeedTask>;
  using ConstPtr = std::shared_ptr<const HasSeedTask>;
  using UPtr = std::unique_ptr<HasSeedTask>;
  using ConstUPtr = std::unique_ptr<const HasSeedTask>;

  HasSeedTask(bool is_conditional = true, std::string name = profile_ns::HAS_SEED_DEFAULT_NAMESPACE);
  ~HasSeedTask() override = default;
  HasSeedTask(const HasSeedTask&) = delete;
  HasSeedTask& operator=(const HasSeedTask&) = delete;
  HasSeedTask(HasSeedTask&&) = delete;
  HasSeedTask& operator=(HasSeedTask&&) = delete;

  TaskComposerNode::UPtr clone() const override final;

  bool operator==(const HasSeedTask& rhs) const;
  bool operator!=(const HasSeedTask& rhs) const;

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  TaskComposerNodeInfo::UPtr runImpl(TaskComposerInput& input,
                                     OptionalTaskComposerExecutor executor = std::nullopt) const override final;
};

}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::HasSeedTask, "HasSeedTask")
#endif  // TESSERACT_TASK_COMPOSER_HAS_SEED_TASK_H
