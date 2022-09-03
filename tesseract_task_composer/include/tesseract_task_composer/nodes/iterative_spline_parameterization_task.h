/**
 * @file iterative_spline_parameterization_task.h
 * @brief Perform iterative spline time parameterization
 *
 * @author Levi Armstrong
 * @date August 11. 2020
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
#ifndef TESSERACT_TASK_COMPOSER_ITERATIVE_SPLINE_PARAMETERIZATION_TASK_H
#define TESSERACT_TASK_COMPOSER_ITERATIVE_SPLINE_PARAMETERIZATION_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/task_composer_task.h>
#include <tesseract_task_composer/nodes/default_task_namespaces.h>
#include <tesseract_time_parameterization/iterative_spline_parameterization.h>

namespace tesseract_planning
{
class IterativeSplineParameterizationTask : public TaskComposerTask
{
public:
  using Ptr = std::shared_ptr<IterativeSplineParameterizationTask>;
  using ConstPtr = std::shared_ptr<const IterativeSplineParameterizationTask>;
  using UPtr = std::unique_ptr<IterativeSplineParameterizationTask>;
  using ConstUPtr = std::unique_ptr<const IterativeSplineParameterizationTask>;

  IterativeSplineParameterizationTask() = default;  // Required for serialization
  IterativeSplineParameterizationTask(
      std::string input_key,
      std::string output_key,
      bool is_conditional = true,
      bool add_points = true,
      std::string name = profile_ns::ITERATIVE_SPLINE_PARAMETERIZATION_DEFAULT_NAMESPACE);

  ~IterativeSplineParameterizationTask() override = default;
  IterativeSplineParameterizationTask(const IterativeSplineParameterizationTask&) = delete;
  IterativeSplineParameterizationTask& operator=(const IterativeSplineParameterizationTask&) = delete;
  IterativeSplineParameterizationTask(IterativeSplineParameterizationTask&&) = delete;
  IterativeSplineParameterizationTask& operator=(IterativeSplineParameterizationTask&&) = delete;

  TaskComposerNode::UPtr clone() const override final;

  bool operator==(const IterativeSplineParameterizationTask& rhs) const;
  bool operator!=(const IterativeSplineParameterizationTask& rhs) const;

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  bool add_points_{ true };
  IterativeSplineParameterization solver_;

  TaskComposerNodeInfo::UPtr runImpl(TaskComposerInput& input,
                                     OptionalTaskComposerExecutor executor = std::nullopt) const override final;
};

}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::IterativeSplineParameterizationTask, "IterativeSplineParameterizationTask")
#endif  // TESSERACT_TASK_COMPOSER_ITERATIVE_SPLINE_PARAMETERIZATION_TASK_H
