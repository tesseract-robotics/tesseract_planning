/**
 * @file utils_test.cpp
 * @brief
 *
 * @author Matthew Powelson
 * @date June 15, 2020
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

#include <tesseract_common/types.h>
#include <tesseract_environment/environment.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/planner_utils.h>
#include <tesseract_support/tesseract_support_resource_locator.h>

using namespace tesseract_planning;
using namespace tesseract_environment;

class TesseractPlanningUtilsUnit : public ::testing::Test
{
protected:
  Environment::Ptr env_;

  void SetUp() override
  {
    auto locator = std::make_shared<tesseract_common::TesseractSupportResourceLocator>();
    Environment::Ptr env = std::make_shared<Environment>();
    tesseract_common::fs::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf");
    tesseract_common::fs::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf");
    EXPECT_TRUE(env->init(urdf_path, srdf_path, locator));
    env_ = env;
  }
};

TEST_F(TesseractPlanningUtilsUnit, GenerateSeed)  // NOLINT
{
  EXPECT_TRUE(true);
}

TEST_F(TesseractPlanningUtilsUnit, GetProfileStringTest)  // NOLINT
{
  std::string input_profile;
  std::string planner_name = "Planner_1";
  std::string default_planner = "TEST_DEFAULT";

  std::unordered_map<std::string, std::string> remap;
  remap["profile_1"] = "profile_1_remapped";
  PlannerProfileRemapping remapping;
  remapping["Planner_2"] = remap;

  // Empty input profile
  std::string output_profile = getProfileString(planner_name, input_profile, remapping);
  EXPECT_EQ(output_profile, "DEFAULT");
  output_profile = getProfileString(planner_name, input_profile, remapping, default_planner);
  EXPECT_EQ(output_profile, default_planner);

  // Planner name doesn't match
  input_profile = "profile_1";
  output_profile = getProfileString(planner_name, input_profile, remapping, default_planner);
  EXPECT_EQ(input_profile, output_profile);

  // Profile name doesn't match
  input_profile = "doesnt_match";
  output_profile = getProfileString("Planner_2", input_profile, remapping, default_planner);
  EXPECT_EQ(input_profile, output_profile);

  // Successful remap
  input_profile = "profile_1";
  output_profile = getProfileString("Planner_2", input_profile, remapping, default_planner);
  EXPECT_EQ(output_profile, "profile_1_remapped");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
