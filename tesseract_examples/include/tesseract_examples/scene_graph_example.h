/**
 * @file scene_graph_example.h
 * @brief This example initializes 2 robots from a URDF. It then reattaches one of the robots to the end effector of the
 * other robot
 *
 * @author Matthew Powelson
 * @date Feb 17, 2020
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
#ifndef TESSERACT_EXAMPLES_SCENE_GRAPH_EXAMPLE_H
#define TESSERACT_EXAMPLES_SCENE_GRAPH_EXAMPLE_H

#include <tesseract_examples/example.h>

namespace tesseract_examples
{
/**
 * @brief Basic example leveraging trajopt and tesseract for cartesian planning
 */
class SceneGraphExample : public Example
{
public:
  SceneGraphExample(tesseract_environment::Environment::Ptr env,
                    tesseract_visualization::Visualization::Ptr plotter = nullptr);
  ~SceneGraphExample() override = default;
  SceneGraphExample(const SceneGraphExample&) = default;
  SceneGraphExample& operator=(const SceneGraphExample&) = default;
  SceneGraphExample(SceneGraphExample&&) = default;
  SceneGraphExample& operator=(SceneGraphExample&&) = default;

  bool run() override final;
};

}  // namespace tesseract_examples

#endif  // TESSERACT_EXAMPLES_SCENE_GRAPH_EXAMPLE_H
