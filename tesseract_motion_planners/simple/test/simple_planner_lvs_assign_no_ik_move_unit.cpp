/**
 * @file simple_planner_lvs_assign_no_ik_move_unit.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date July 28, 2020
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
#include "simple_planner_test_utils.hpp"

#include <gtest/gtest.h>
#include <tesseract_common/types.h>
#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_assign_no_ik_move_profile.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/move_instruction.h>

using namespace tesseract_planning;

class TesseractPlanningSimplePlannerLVSAssignNoIKMoveProfileUnit : public TesseractPlanningSimplePlannerUnit
{
};

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
