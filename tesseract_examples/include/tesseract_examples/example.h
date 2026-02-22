/**
 * @file examples.h
 * @brief Examples base class
 *
 * @author Levi Armstrong
 * @date July 22, 2018
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_ROS_EXAMPLES_EXAMPLES_H
#define TESSERACT_ROS_EXAMPLES_EXAMPLES_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/fwd.h>
#include <tesseract_visualization/fwd.h>

namespace tesseract::examples
{
/**
 * @brief The Example base class
 *
 * It provides a generic interface for all examples as a library which then
 * can easily be integrated as unit tests so breaking changes are caught.
 * Also it provides a few utility functions for checking rviz environment and
 * updating the rviz environment.
 */
class Example
{
public:
  Example(std::shared_ptr<tesseract::environment::Environment> env,
          std::shared_ptr<tesseract::visualization::Visualization> plotter = nullptr);

  virtual ~Example() = default;
  Example(const Example&) = default;
  Example& operator=(const Example&) = default;
  Example(Example&&) = default;
  Example& operator=(Example&&) = default;

  virtual bool run() = 0;

protected:
  /** @brief Tesseract Manager Class (Required) */
  std::shared_ptr<tesseract::environment::Environment> env_;
  /** @brief Tesseract Visualization Class (Optional)*/
  std::shared_ptr<tesseract::visualization::Visualization> plotter_;
};

}  // namespace tesseract::examples
#endif  // TESSERACT_ROS_EXAMPLES_EXAMPLES_H
