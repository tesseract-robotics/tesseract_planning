/**
 * @file Validate task composer config

 * @author Levi Armstrong
 * @date Dec 9, 2024
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2024, Southwest Research Institute
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/program_options.hpp>
#include <iostream>
#include <filesystem>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/resource_locator.h>
#include <tesseract_task_composer/core/task_composer_server.h>
#include <tesseract_task_composer/core/task_composer_node.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>

namespace
{
constexpr int kErrorInCommandLine = 1;
constexpr int kSuccess = 0;
constexpr int kErrorUnhandledException = 2;
}  // namespace

int main(int argc, char** argv)
{
  std::string input;
  std::string output;

  namespace po = boost::program_options;
  po::options_description desc("Options");
  desc.add_options()("help,h", "Print help messages")(
      "input,i", po::value<std::string>(&input)->required(), "File path to task composer config.")(
      "dump,d",
      po::value<std::string>(&output)->default_value(""),
      "The directory location to save the generated dotgraphs.");

  po::variables_map vm;
  try
  {
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help") > 0)
    {
      std::cout << "Task Composer Validate Config Tool\n" << desc << "\n";
      return kSuccess;
    }

    po::notify(vm);
  }
  catch (const po::error& e)
  {
    std::cerr << "ERROR: " << e.what() << "\n\n" << desc << "\n";
    return kErrorInCommandLine;
  }

  const std::filesystem::path config(input);
  if (!std::filesystem::exists(config))
  {
    std::cerr << "Input file does not exist: " << config.string() << "\n";
    return kErrorInCommandLine;
  }

  const std::filesystem::path dump_location(output);
  if (!output.empty() && !std::filesystem::exists(dump_location))
  {
    std::cerr << "Dump location does not exist: " << dump_location.string() << "\n";
    return kErrorInCommandLine;
  }

  tesseract::common::GeneralResourceLocator locator;
  tesseract::task_composer::TaskComposerServer server;
  try
  {
    server.loadConfig(config, locator);
  }
  catch (const std::exception& ex)
  {
    std::cerr << "Planning server failed to load config: " << ex.what() << "\n";
    return kErrorUnhandledException;
  }

  if (output.empty())
    return kSuccess;

  for (const auto& task_name : server.getAvailableTasks())
  {
    auto task_filepath = dump_location;
    task_filepath /= (task_name + ".dot");

    const auto& task = server.getTask(task_name);
    task.saveDotgraph(task_filepath.string());
  }

  std::cout << "Task Dotgraphs saved to: " << dump_location.string() << "\n";
  return kSuccess;
}
