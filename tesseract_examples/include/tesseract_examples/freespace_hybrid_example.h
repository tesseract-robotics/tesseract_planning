/**
 * @file freespace_hybrid_example.h
 * @brief An example of a feespace motion planning with OMPL then TrajOpt
 *
 * @author Levi Armstrong
 * @date March 16, 2020
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
#ifndef TESSERACT_EXAMPLES_FREESPACE_HYBRID_EXAMPLE_H
#define TESSERACT_EXAMPLES_FREESPACE_HYBRID_EXAMPLE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_examples/example.h>

namespace tesseract_examples
{
/**
 * @brief An example of a robot leveraging OMPL RRTConnect to generate a freespace motion trajectory the TrajOpt to
 * optimize
 */
class FreespaceHybridExample : public Example
{
public:
  FreespaceHybridExample(tesseract_environment::Environment::Ptr env,
                         tesseract_visualization::Visualization::Ptr plotter = nullptr,
                         double range = 0.01,
                         double planning_time = 60.0);
  ~FreespaceHybridExample() override = default;
  FreespaceHybridExample(const FreespaceHybridExample&) = default;
  FreespaceHybridExample& operator=(const FreespaceHybridExample&) = default;
  FreespaceHybridExample(FreespaceHybridExample&&) = default;
  FreespaceHybridExample& operator=(FreespaceHybridExample&&) = default;

  bool run() override final;

private:
  double range_;
  double planning_time_;

  static tesseract_environment::Command::Ptr addSphere();
};

}  // namespace tesseract_examples

#endif  // TESSERACT_EXAMPLES_FREESPACE_HYBRID_EXAMPLE_H
