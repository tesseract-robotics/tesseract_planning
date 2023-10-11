/**
 * @file utils.h
 * @brief Planner utility functions.
 *
 * @author Levi Armstrong
 * @date April 18, 2018
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
#ifndef TESSERACT_MOTION_PLANNERS_UTILS_H
#define TESSERACT_MOTION_PLANNERS_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/environment.h>
#include <tesseract_environment/utils.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_command_language/poly/instruction_poly.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_common/types.h>

namespace tesseract_planning
{
/**
 * @brief Extract toolpath from a instruction
 * @param instruction The instruction to extract toolpath
 * @param env The environment object used for getting kinematics and tcp information
 * @return A toolpath in world coordinate system
 */
tesseract_common::Toolpath toToolpath(const InstructionPoly& instruction,
                                      const tesseract_environment::Environment& env);

/**
 * @brief Extract toolpath from a composite instruction
 * @param instruction The instruction to extract toolpath
 * @param env The environment object used for getting kinematics and tcp information
 * @return A toolpath in world coordinate system
 */
tesseract_common::Toolpath toToolpath(const CompositeInstruction& ci, const tesseract_environment::Environment& env);

/**
 * @brief Extract toolpath from a move instruction
 * @param instruction The instruction to extract toolpath
 * @param env The environment object used for getting kinematics and tcp information
 * @return A toolpath in world coordinate system
 */
tesseract_common::Toolpath toToolpath(const MoveInstructionPoly& mi, const tesseract_environment::Environment& env);

/**
 * @brief This will assign the current state as the seed for all cartesian waypoint
 * @param composite_instructions The input program
 * @param env The environment information
 */
void assignCurrentStateAsSeed(CompositeInstruction& composite_instructions,
                              const tesseract_environment::Environment& env);

/**
 * @brief This formats the joint and state waypoints to align with the kinematics object
 * @param composite_instructions The input program to format
 * @param env The environment information
 * @return True if the program required formatting.
 */
bool formatProgram(CompositeInstruction& composite_instructions, const tesseract_environment::Environment& env);

/**
 * @brief Should perform a continuous collision check over the trajectory.
 * @param contacts A vector of vector of ContactMap where each index corresponds to a timestep
 * @param manager A continuous contact manager
 * @param state_solver The environment state solver
 * @param program The program to check for contacts
 * @param config CollisionCheckConfig used to specify collision check settings
 * @return True if collision was found, otherwise false.
 */
bool contactCheckProgram(std::vector<tesseract_collision::ContactResultMap>& contacts,
                         tesseract_collision::ContinuousContactManager& manager,
                         const tesseract_scene_graph::StateSolver& state_solver,
                         const CompositeInstruction& program,
                         const tesseract_collision::CollisionCheckConfig& config);

/**
 * @brief Should perform a discrete collision check over the trajectory
 * @param contacts A vector of vector of ContactMap where each index corresponds to a timestep
 * @param manager A continuous contact manager
 * @param state_solver The environment state solver
 * @param program The program to check for contacts
 * @param config CollisionCheckConfig used to specify collision check settings
 * @return True if collision was found, otherwise false.
 */
bool contactCheckProgram(std::vector<tesseract_collision::ContactResultMap>& contacts,
                         tesseract_collision::DiscreteContactManager& manager,
                         const tesseract_scene_graph::StateSolver& state_solver,
                         const CompositeInstruction& program,
                         const tesseract_collision::CollisionCheckConfig& config);

}  // namespace tesseract_planning

#endif  // TESSERACT_PLANNING_UTILS_H
