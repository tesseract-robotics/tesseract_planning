/**
 * @file glass_upright_example.h
 * @brief An example of a robot with fixed orientation but free to move in cartesian space.
 *
 * @author Levi Armstrong
 * @date July 22, 2018
 * @version TODO
 * @bug No known bugs
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
#ifndef TESSERACT_EXAMPLES_GLASS_UPRIGHT_EXAMPLE_H
#define TESSERACT_EXAMPLES_GLASS_UPRIGHT_EXAMPLE_H

#include <tesseract_common/macros.h>

#include <tesseract_examples/example.h>

namespace tesseract_examples
{
/**
 * @brief An example of a robot with fixed orientation but free to move in cartesian space
 * leveraging tesseract and trajopt to generate a motion trajectory.
 */
class GlassUprightExample : public Example
{
public:
  GlassUprightExample(tesseract_environment::Environment::Ptr env,
                      tesseract_visualization::Visualization::Ptr plotter = nullptr,
                      bool ifopt = false,
                      bool debug = false);
  ~GlassUprightExample() override = default;
  GlassUprightExample(const GlassUprightExample&) = default;
  GlassUprightExample& operator=(const GlassUprightExample&) = default;
  GlassUprightExample(GlassUprightExample&&) = default;
  GlassUprightExample& operator=(GlassUprightExample&&) = default;

  bool run() override final;

private:
  bool ifopt_;
  bool debug_;
  static tesseract_environment::Command::Ptr addSphere();
};

}  // namespace tesseract_examples

#endif  // TESSERACT_EXAMPLES_GLASS_UPRIGHT_EXAMPLE_H
