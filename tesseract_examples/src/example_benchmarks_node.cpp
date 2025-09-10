/**
 * @file example_benchmarks_node.cpp
 * @brief Example benchmarks
 *
 * @author Levi Armstrong
 * @date July 22, 2019
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

#include <tesseract_examples/basic_cartesian_example.h>
#include <tesseract_examples/freespace_ompl_example.h>
#include <tesseract_examples/glass_upright_example.h>
#include <tesseract_examples/puzzle_piece_example.h>
#include <tesseract_examples/puzzle_piece_auxillary_axes_example.h>

#include <filesystem>
#include <console_bridge/console.h>
#include <tesseract_environment/environment.h>
#include <tesseract_common/resource_locator.h>

using namespace tesseract_examples;
using namespace tesseract_common;
using namespace tesseract_environment;

int main(int /*argc*/, char** /*argv*/)
{
  auto locator = std::make_shared<GeneralResourceLocator>();

  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);
  CONSOLE_BRIDGE_logInform("Example, Contact Manager, First Solve (s), Remaining Solve Avg. (s), Count");

  {
    std::filesystem::path urdf_path =
        locator->locateResource("package://tesseract_support/urdf/lbr_iiwa_14_r820.urdf")->getFilePath();
    std::filesystem::path srdf_path =
        locator->locateResource("package://tesseract_support/urdf/lbr_iiwa_14_r820.srdf")->getFilePath();
    auto env = std::make_shared<Environment>();
    if (!env->init(urdf_path, srdf_path, locator))
      exit(1);

    BasicCartesianExample example(env, nullptr, false, false, true);
    if (!example.run())
    {
      CONSOLE_BRIDGE_logError("BasicCartesianExample failed");
      exit(1);
    }
  }

  {
    std::filesystem::path urdf_path =
        locator->locateResource("package://tesseract_support/urdf/lbr_iiwa_14_r820.urdf")->getFilePath();
    std::filesystem::path srdf_path =
        locator->locateResource("package://tesseract_support/urdf/lbr_iiwa_14_r820.srdf")->getFilePath();
    auto env = std::make_shared<Environment>();
    if (!env->init(urdf_path, srdf_path, locator))
      exit(1);

    FreespaceOMPLExample example(env, nullptr, 0.01, 60.0, false, true);
    if (!example.run())
    {
      CONSOLE_BRIDGE_logError("FreespaceOMPLExample failed");
      exit(1);
    }
  }

  {
    std::filesystem::path urdf_path =
        locator->locateResource("package://tesseract_support/urdf/lbr_iiwa_14_r820.urdf")->getFilePath();
    std::filesystem::path srdf_path =
        locator->locateResource("package://tesseract_support/urdf/lbr_iiwa_14_r820.srdf")->getFilePath();
    auto env = std::make_shared<Environment>();
    if (!env->init(urdf_path, srdf_path, locator))
      exit(1);

    GlassUprightExample example(env, nullptr, false, false, true);
    if (!example.run())
    {
      CONSOLE_BRIDGE_logError("GlassUprightExample failed");
      exit(1);
    }
  }

  {
    std::filesystem::path urdf_path =
        locator->locateResource("package://tesseract_support/urdf/puzzle_piece_workcell.urdf")->getFilePath();
    std::filesystem::path srdf_path =
        locator->locateResource("package://tesseract_support/urdf/puzzle_piece_workcell.srdf")->getFilePath();
    auto env = std::make_shared<Environment>();
    if (!env->init(urdf_path, srdf_path, locator))
      exit(1);

    PuzzlePieceExample example(env, nullptr, false, false, true);
    if (!example.run())
    {
      CONSOLE_BRIDGE_logError("PuzzlePieceExample failed");
      exit(1);
    }
  }

  {
    std::filesystem::path urdf_path =
        locator->locateResource("package://tesseract_support/urdf/puzzle_piece_workcell.urdf")->getFilePath();
    std::filesystem::path srdf_path =
        locator->locateResource("package://tesseract_support/urdf/puzzle_piece_workcell.srdf")->getFilePath();
    auto env = std::make_shared<Environment>();
    if (!env->init(urdf_path, srdf_path, locator))
      exit(1);

    PuzzlePieceAuxillaryAxesExample example(env, nullptr, false, false, true);
    if (!example.run())
    {
      CONSOLE_BRIDGE_logError("PuzzlePieceAuxillaryAxesExample failed");
      exit(1);
    }
  }
}
