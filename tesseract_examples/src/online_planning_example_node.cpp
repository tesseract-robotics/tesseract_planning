/**
 * @file online_planning_example_node.cpp
 * @brief This example demonstrates using trajopt to plan in an "online" manner. As the environment is changed (using
 * the joint state publisher), the system will attempt to avoid collisions with the moving object and follow the moving
 * target
 *
 * Note: If the target moves too quickly the solver can get stuck in an infeasible point. That is the nature of how the
 * solver is working. Higher level intelligence, a larger step size, or changing the target to a cost can solve that
 * problem.
 *
 * @author Matthew Powelson
 * @date June 9, 2020
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

#include <tesseract_examples/online_planning_example.h>
#include <filesystem>
#include <console_bridge/console.h>
#include <tesseract/environment/environment.h>
#include <tesseract/common/resource_locator.h>

using namespace tesseract::examples;
using namespace tesseract::common;
using namespace tesseract::environment;

int main(int /*argc*/, char** /*argv*/)
{
  auto locator = std::make_shared<GeneralResourceLocator>();
  std::filesystem::path urdf_path =
      locator->locateResource("package://tesseract/support/urdf/online_planning_example.urdf")->getFilePath();
  std::filesystem::path srdf_path =
      locator->locateResource("package://tesseract/support/urdf/online_planning_example.srdf")->getFilePath();
  auto env = std::make_shared<Environment>();
  if (!env->init(urdf_path, srdf_path, locator))
    exit(1);

  OnlinePlanningExample example(env, nullptr);
  if (!example.run())
  {
    CONSOLE_BRIDGE_logError("OnlinePlanningExample failed");
    exit(1);
  }

  CONSOLE_BRIDGE_logInform("OnlinePlanningExample successful");
}
