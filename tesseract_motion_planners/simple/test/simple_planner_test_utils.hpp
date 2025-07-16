/**
 * @file simple_planner_test_utils.cpp
 * @brief
 *
 * @author Matthew Powelson
 * @date July 23, 2020
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_environment/environment.h>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/resource_locator.h>

using namespace tesseract_environment;

class TesseractPlanningSimplePlannerUnit : public ::testing::Test
{
protected:
  Environment::Ptr env_;
  tesseract_common::ManipulatorInfo manip_info_;
  std::vector<std::string> joint_names_;

  void SetUp() override
  {
    auto locator = std::make_shared<tesseract_common::GeneralResourceLocator>();
    Environment::Ptr env = std::make_shared<Environment>();
    std::filesystem::path urdf_path(
        locator->locateResource("package://tesseract_support/urdf/lbr_iiwa_14_r820.urdf")->getFilePath());
    std::filesystem::path srdf_path(
        locator->locateResource("package://tesseract_support/urdf/lbr_iiwa_14_r820.srdf")->getFilePath());
    EXPECT_TRUE(env->init(urdf_path, srdf_path, locator));
    env_ = env;

    manip_info_.manipulator = "manipulator";
    manip_info_.tcp_frame = "tool0";
    manip_info_.working_frame = "base_link";
    joint_names_ = env_->getJointGroup("manipulator")->getJointNames();
  }
};
