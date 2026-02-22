/**
 * @file utils.h
 * @brief Planner utility functions.
 *
 * @author Levi Armstrong
 * @date April 18, 2018
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
#ifndef TESSERACT_MOTION_PLANNERS_UTILS_H
#define TESSERACT_MOTION_PLANNERS_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/eigen_types.h>

#include <tesseract_collision/core/fwd.h>
#include <tesseract_state_solver/fwd.h>
#include <tesseract_environment/fwd.h>
#include <tesseract_command_language/fwd.h>

namespace tesseract::motion_planners
{
/**
 * @brief Extract toolpath from a instruction
 * @param instruction The instruction to extract toolpath
 * @param env The environment object used for getting kinematics and tcp information
 * @return A toolpath in world coordinate system
 */
tesseract::common::Toolpath toToolpath(const tesseract::command_language::InstructionPoly& instruction,
                                       const tesseract::environment::Environment& env);

/**
 * @brief Extract toolpath from a composite instruction
 * @param instruction The instruction to extract toolpath
 * @param env The environment object used for getting kinematics and tcp information
 * @return A toolpath in world coordinate system
 */
tesseract::common::Toolpath toToolpath(const tesseract::command_language::CompositeInstruction& ci,
                                       const tesseract::environment::Environment& env);

/**
 * @brief Extract toolpath from a move instruction
 * @param instruction The instruction to extract toolpath
 * @param env The environment object used for getting kinematics and tcp information
 * @return A toolpath in world coordinate system
 */
tesseract::common::Toolpath toToolpath(const tesseract::command_language::MoveInstructionPoly& mi,
                                       const tesseract::environment::Environment& env);

/**
 * @brief This will assign the current state as the seed for all cartesian waypoint
 * @param composite_instructions The input program
 * @param env The environment information
 */
void assignCurrentStateAsSeed(tesseract::command_language::CompositeInstruction& composite_instructions,
                              const tesseract::environment::Environment& env);

/**
 * @brief This formats the joint and state waypoints to align with the kinematics object
 * @param composite_instructions The input program to format
 * @param env The environment information
 * @return True if the program required formatting.
 */
bool formatProgram(tesseract::command_language::CompositeInstruction& composite_instructions,
                   const tesseract::environment::Environment& env);

/**
 * @brief Should perform a continuous collision check over the trajectory.
 * @param contacts A vector of vector of ContactMap where each index corresponds to a timestep
 * @param manager A continuous contact manager
 * @param state_solver The environment state solver
 * @param program The program to check for contacts
 * @param config CollisionCheckConfig used to specify collision check settings
 * @return ContactTrajectoryResults containing contact step/substep locations and joint values.
 */
tesseract::collision::ContactTrajectoryResults
contactCheckProgram(std::vector<tesseract::collision::ContactResultMap>& contacts,
                    tesseract::collision::ContinuousContactManager& manager,
                    const tesseract::scene_graph::StateSolver& state_solver,
                    const tesseract::command_language::CompositeInstruction& program,
                    const tesseract::collision::CollisionCheckConfig& config);

/**
 * @brief Should perform a discrete collision check over the trajectory
 * @param contacts A vector of vector of ContactMap where each index corresponds to a timestep
 * @param manager A continuous contact manager
 * @param state_solver The environment state solver
 * @param program The program to check for contacts
 * @param config CollisionCheckConfig used to specify collision check settings
 * @return ContactTrajectoryResults containing contact step/substep locations and joint values.
 */
tesseract::collision::ContactTrajectoryResults
contactCheckProgram(std::vector<tesseract::collision::ContactResultMap>& contacts,
                    tesseract::collision::DiscreteContactManager& manager,
                    const tesseract::scene_graph::StateSolver& state_solver,
                    const tesseract::command_language::CompositeInstruction& program,
                    const tesseract::collision::CollisionCheckConfig& config);

}  // namespace tesseract::motion_planners

#endif  // TESSERACT_PLANNING_UTILS_H
