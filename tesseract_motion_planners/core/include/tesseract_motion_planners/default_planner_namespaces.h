/**
 * @file default_planner_namespaces.h
 * @brief A collection of motion planner profile namespaces
 *
 * @author Levi Armstrong
 * @date June 18, 2020
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
#ifndef TESSERACT_MOTION_PLANNERS_DEFAULT_PLANNER_NAMESPACES_H
#define TESSERACT_MOTION_PLANNERS_DEFAULT_PLANNER_NAMESPACES_H

namespace tesseract_planning::profile_ns
{
// clang-format off
static const std::string SIMPLE_DEFAULT_NAMESPACE        = "SIMPLE_PLANNER";
static const std::string DESCARTES_DEFAULT_NAMESPACE     = "DESCARTES";
static const std::string OMPL_DEFAULT_NAMESPACE          = "OMPL";
static const std::string TRAJOPT_DEFAULT_NAMESPACE       = "TRAJOPT";
static const std::string TRAJOPT_IFOPT_DEFAULT_NAMESPACE = "TRAJOPT_IFOPT";
static const std::string TRAJOPT_CONSTRAINT_NAMESPACE = "TRAJOPT_CONSTRAINT";
// clang-format on
}  // namespace tesseract_planning::profile_ns
#endif  // TESSERACT_MOTION_PLANNERS_DEFAULT_PLANNER_NAMESPACES_H
