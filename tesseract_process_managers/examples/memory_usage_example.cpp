/**
 * @file memory_usage_example.cpp
 * @brief This example it to evaluate the memory the usage of the planning server
 *
 * @author Levi Armstrong
 * @date January 15, 2021
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <fstream>
#include <chrono>
#include <unistd.h>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "freespace_example_program.h"

#include <tesseract_common/types.h>
#include <tesseract_environment/environment.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_process_managers/core/process_planning_server.h>
#include <tesseract_visualization/visualization_loader.h>

using namespace tesseract_planning;

std::string locateResource(const std::string& url)
{
  std::string mod_url = url;
  if (url.find("package://tesseract_support") == 0)
  {
    mod_url.erase(0, strlen("package://tesseract_support"));
    size_t pos = mod_url.find('/');
    if (pos == std::string::npos)
    {
      return std::string();
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = std::string(TESSERACT_SUPPORT_DIR);

    if (package_path.empty())
    {
      return std::string();
    }

    mod_url = package_path + mod_url;
  }

  return mod_url;
}

//////////////////////////////////////////////////////////////////////////////
//
// process_mem_usage(double &, double &) - takes two doubles by reference,
// attempts to read the system-dependent data for a process' virtual memory
// size and resident set size, and return the results in KB.
//
// On failure, returns 0.0, 0.0

/**
 * @brief Extract process memory usage
 *
 * @details attempts to read the system-dependent data for a process' virtual memory
 * size and resident set size, and return the results in KB.
 *
 * On failure, returns 0.0, 0.0
 *
 * @param vm_usage The virtual memory being used by the process (KB)
 * @param resident_set The physical RAM being used by the process (KB)
 */
void process_mem_usage(double& vm_usage, double& resident_set)
{
  using std::ifstream;
  using std::ios_base;
  using std::string;

  vm_usage = 0.0;
  resident_set = 0.0;

  // 'file' stat seems to give the most reliable results
  //
  ifstream stat_stream("/proc/self/stat", ios_base::in);

  // dummy vars for leading entries in stat that we don't care about
  //
  std::string pid, comm, state, ppid, pgrp, session, tty_nr;
  std::string tpgid, flags, minflt, cminflt, majflt, cmajflt;
  std::string utime, stime, cutime, cstime, priority, nice;
  std::string O, itrealvalue, starttime;

  // the two fields we want
  //
  unsigned long vsize{ 0 };
  long rss{ 0 };

  stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr >> tpgid >> flags >> minflt >> cminflt >>
      majflt >> cmajflt >> utime >> stime >> cutime >> cstime >> priority >> nice >> O >> itrealvalue >> starttime >>
      vsize >> rss;  // don't care about the rest

  stat_stream.close();

  long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024;  // in case x86-64 is configured to use 2MB pages
  vm_usage = static_cast<double>(vsize) / 1024.0;
  resident_set = static_cast<double>(rss * page_size_kb);
}

ProcessPlanningRequest getPlanningRequest()
{
  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = process_planner_names::TRAJOPT_PLANNER_NAME;

  // Define the program
  CompositeInstruction program = freespaceExampleProgramIIWA();
  request.instructions = Instruction(program);

  return request;
}

int main()
{
  // --------------------
  // Perform setup
  // --------------------
  auto locator = std::make_shared<tesseract_common::SimpleResourceLocator>(locateResource);
  tesseract_environment::Environment::Ptr env = std::make_shared<tesseract_environment::Environment>();
  tesseract_common::fs::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf");
  tesseract_common::fs::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf");
  env->init(urdf_path, srdf_path, locator);

  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env), 5);
  planning_server.loadDefaultProcessPlanners();

  // Solve process plan
  using Clock = std::chrono::high_resolution_clock;
  auto t1 = Clock::now();
  auto t2 = Clock::now();
  double vm{ NAN }, rss{ NAN };
  while (std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() < 3600)
  {
    ProcessPlanningRequest request1 = getPlanningRequest();
    ProcessPlanningFuture response1 = planning_server.run(request1);
    ProcessPlanningRequest request2 = getPlanningRequest();
    ProcessPlanningFuture response2 = planning_server.run(request2);
    ProcessPlanningRequest request3 = getPlanningRequest();
    ProcessPlanningFuture response3 = planning_server.run(request3);
    ProcessPlanningRequest request4 = getPlanningRequest();
    ProcessPlanningFuture response4 = planning_server.run(request4);
    ProcessPlanningRequest request5 = getPlanningRequest();
    ProcessPlanningFuture response5 = planning_server.run(request5);
    planning_server.waitForAll();
    process_mem_usage(vm, rss);
    std::cout << "VM: " << vm << " KB; RSS: " << rss << " KB" << std::endl;
  }

  std::cout << "Execution Complete" << std::endl;

  return 0;
}
