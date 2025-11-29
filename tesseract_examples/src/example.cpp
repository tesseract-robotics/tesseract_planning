/**
 * @file examples.cpp
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

#include <tesseract_examples/example.h>

#include <tesseract_environment/environment.h>
#include <tesseract_visualization/visualization.h>

namespace tesseract_examples
{
Example::Example(std::shared_ptr<tesseract_environment::Environment> env,
                 std::shared_ptr<tesseract_visualization::Visualization> plotter)
  : env_(std::move(env)), plotter_(std::move(plotter))
{
}
}  // namespace tesseract_examples
