/**
 * @file yaml_extensions.h
 * @brief YAML Type conversions for OMPL types
 *
 * @author Samantha Smith
 * @date July 29, 2025
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2025, Southwest Research Institute
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
#ifndef TESSERACT_MOTION_PLANNING_YAML_EXTENSIONS_H
#define TESSERACT_MOTION_PLANNING_YAML_EXTENSIONS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <yaml-cpp/yaml.h>
#include <set>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/plugin_info.h>
#include <tesseract_motion_planners/ompl/ompl_solver_config.h>
#include <tesseract_motion_planners/ompl/profile/ompl_real_vector_move_profile.h>
#include <tesseract_motion_planners/ompl/ompl_planner_configurator.h>
#include <tesseract_common/profile_plugin_factory.h>

namespace YAML
{
//=========================== SBLConfigurator ===========================
template <>
struct convert<tesseract_planning::SBLConfigurator>
{
  static Node encode(const tesseract_planning::SBLConfigurator& rhs)
  {
    Node node;
    node["range"] = rhs.range;
    return node;
  }

  static bool decode(const Node& node, tesseract_planning::SBLConfigurator& rhs)
  {
    // Check for required entries
    if (const YAML::Node& n = node["range"])
      rhs.range = n.as<double>();
    return true;
  }
};

//=========================== ESTConfigurator ===========================
template <>
struct convert<tesseract_planning::ESTConfigurator>
{
  static Node encode(const tesseract_planning::ESTConfigurator& rhs)
  {
    Node node;
    node["range"] = rhs.range;
    node["goal_bias"] = rhs.goal_bias;
    return node;
  }

  static bool decode(const Node& node, tesseract_planning::ESTConfigurator& rhs)
  {
    // Check for required entries
    if (const YAML::Node& n = node["range"])
      rhs.range = n.as<double>();
    if (const YAML::Node& n = node["goal_bias"])
      rhs.goal_bias = n.as<double>();
    return true;
  }
};

//=========================== LBKPIECE1Configurator ===========================
template <>
struct convert<tesseract_planning::LBKPIECE1Configurator>
{
  static Node encode(const tesseract_planning::LBKPIECE1Configurator& rhs)
  {
    Node node;
    node["range"] = rhs.range;
    node["border_fraction"] = rhs.border_fraction;
    node["min_valid_path_fraction"] = rhs.min_valid_path_fraction;
    return node;
  }

  static bool decode(const Node& node, tesseract_planning::LBKPIECE1Configurator& rhs)
  {
    // Check for required entries
    if (const YAML::Node& n = node["range"])
      rhs.range = n.as<double>();
    if (const YAML::Node& n = node["border_fraction"])
      rhs.border_fraction = n.as<double>();
    if (const YAML::Node& n = node["min_valid_path_fraction"])
      rhs.min_valid_path_fraction = n.as<double>();
    return true;
  }
};

//=========================== BKPIECE1Configurator ===========================
template <>
struct convert<tesseract_planning::BKPIECE1Configurator>
{
  static Node encode(const tesseract_planning::BKPIECE1Configurator& rhs)
  {
    Node node;
    node["range"] = rhs.range;
    node["border_fraction"] = rhs.border_fraction;
    node["failed_expansion_score_factor"] = rhs.failed_expansion_score_factor;
    node["min_valid_path_fraction"] = rhs.min_valid_path_fraction;
    return node;
  }

  static bool decode(const Node& node, tesseract_planning::BKPIECE1Configurator& rhs)
  {
    // Check for required entries
    if (const YAML::Node& n = node["range"])
      rhs.range = n.as<double>();
    if (const YAML::Node& n = node["border_fraction"])
      rhs.border_fraction = n.as<double>();
    if (const YAML::Node& n = node["min_valid_path_fraction"])
      rhs.min_valid_path_fraction = n.as<double>();
    if (const YAML::Node& n = node["failed_expansion_score_factor"])
      rhs.failed_expansion_score_factor = n.as<double>();
    return true;
  }
};

//=========================== KPIECE1Configurator ===========================
template <>
struct convert<tesseract_planning::KPIECE1Configurator>
{
  static Node encode(const tesseract_planning::KPIECE1Configurator& rhs)
  {
    Node node;
    node["range"] = rhs.range;
    node["goal_bias"] = rhs.goal_bias;
    node["border_fraction"] = rhs.border_fraction;
    node["failed_expansion_score_factor"] = rhs.failed_expansion_score_factor;
    node["min_valid_path_fraction"] = rhs.min_valid_path_fraction;
    return node;
  }

  static bool decode(const Node& node, tesseract_planning::KPIECE1Configurator& rhs)
  {
    // Check for required entries
    if (const YAML::Node& n = node["range"])
      rhs.range = n.as<double>();
    if (const YAML::Node& n = node["border_fraction"])
      rhs.border_fraction = n.as<double>();
    if (const YAML::Node& n = node["min_valid_path_fraction"])
      rhs.min_valid_path_fraction = n.as<double>();
    if (const YAML::Node& n = node["failed_expansion_score_factor"])
      rhs.failed_expansion_score_factor = n.as<double>();
    if (const YAML::Node& n = node["goal_bias"])
      rhs.goal_bias = n.as<double>();
    return true;
  }
};

//=========================== BiTRRTConfigurator ===========================
template <>
struct convert<tesseract_planning::BiTRRTConfigurator>
{
  static Node encode(const tesseract_planning::BiTRRTConfigurator& rhs)
  {
    Node node;
    node["range"] = rhs.range;
    node["temp_change_factor"] = rhs.temp_change_factor;
    node["cost_threshold"] = rhs.cost_threshold;
    node["init_temperature"] = rhs.init_temperature;
    node["frontier_threshold"] = rhs.frontier_threshold;
    node["frontier_node_ratio"] = rhs.frontier_node_ratio;

    return node;
  }

  static bool decode(const Node& node, tesseract_planning::BiTRRTConfigurator& rhs)
  {
    // Check for required entries
    if (const YAML::Node& n = node["range"])
      rhs.range = n.as<double>();
    if (const YAML::Node& n = node["temp_change_factor"])
      rhs.temp_change_factor = n.as<double>();
    if (const YAML::Node& n = node["cost_threshold"])
      rhs.cost_threshold = n.as<double>();
    if (const YAML::Node& n = node["init_temperature"])
      rhs.init_temperature = n.as<double>();
    if (const YAML::Node& n = node["frontier_threshold"])
      rhs.frontier_threshold = n.as<double>();
    if (const YAML::Node& n = node["frontier_node_ratio"])
      rhs.frontier_node_ratio = n.as<double>();
    return true;
  }
};

//=========================== RRTConfigurator ===========================
template <>
struct convert<tesseract_planning::RRTConfigurator>
{
  static Node encode(const tesseract_planning::RRTConfigurator& rhs)
  {
    Node node;
    node["range"] = rhs.range;
    node["goal_bias"] = rhs.goal_bias;
    return node;
  }

  static bool decode(const Node& node, tesseract_planning::RRTConfigurator& rhs)
  {
    // Check for required entries
    if (const YAML::Node& n = node["range"])
      rhs.range = n.as<double>();
    if (const YAML::Node& n = node["goal_bias"])
      rhs.goal_bias = n.as<double>();
    return true;
  }
};

//=========================== RRTConnectConfigurator ===========================
template <>
struct convert<tesseract_planning::RRTConnectConfigurator>
{
  static Node encode(const tesseract_planning::RRTConnectConfigurator& rhs)
  {
    Node node;
    node["range"] = rhs.range;
    return node;
  }

  static bool decode(const Node& node, tesseract_planning::RRTConnectConfigurator& rhs)
  {
    // Check for required entries
    if (const YAML::Node& n = node["range"])
      rhs.range = n.as<double>();
    return true;
  }
};

//=========================== RRTstarConfigurator ===========================
template <>
struct convert<tesseract_planning::RRTstarConfigurator>
{
  static Node encode(const tesseract_planning::RRTstarConfigurator& rhs)
  {
    Node node;
    node["range"] = rhs.range;
    node["goal_bias"] = rhs.goal_bias;
    node["delay_collision_checking"] = rhs.delay_collision_checking;
    return node;
  }

  static bool decode(const Node& node, tesseract_planning::RRTstarConfigurator& rhs)
  {
    // Check for required entries
    if (const YAML::Node& n = node["range"])
      rhs.range = n.as<double>();
    if (const YAML::Node& n = node["goal_bias"])
      rhs.goal_bias = n.as<double>();
    if (const YAML::Node& n = node["delay_collision_checking"])
      rhs.delay_collision_checking = n.as<bool>();
    return true;
  }
};

//=========================== TRRTConfigurator ===========================
template <>
struct convert<tesseract_planning::TRRTConfigurator>
{
  static Node encode(const tesseract_planning::TRRTConfigurator& rhs)
  {
    Node node;
    node["range"] = rhs.range;
    node["goal_bias"] = rhs.goal_bias;
    node["temp_change_factor"] = rhs.temp_change_factor;
    node["init_temperature"] = rhs.init_temperature;
    node["frontier_threshold"] = rhs.frontier_threshold;
    node["frontier_node_ratio"] = rhs.frontier_node_ratio;

    return node;
  }

  static bool decode(const Node& node, tesseract_planning::TRRTConfigurator& rhs)
  {
    // Check for required entries
    if (const YAML::Node& n = node["range"])
      rhs.range = n.as<double>();
    if (const YAML::Node& n = node["goal_bias"])
      rhs.goal_bias = n.as<double>();
    if (const YAML::Node& n = node["temp_change_factor"])
      rhs.temp_change_factor = n.as<double>();
    if (const YAML::Node& n = node["init_temperature"])
      rhs.init_temperature = n.as<double>();
    if (const YAML::Node& n = node["frontier_threshold"])
      rhs.frontier_threshold = n.as<double>();
    if (const YAML::Node& n = node["frontier_node_ratio"])
      rhs.frontier_node_ratio = n.as<double>();
    return true;
  }
};

//=========================== PRMConfigurator ===========================
template <>
struct convert<tesseract_planning::PRMConfigurator>
{
  static Node encode(const tesseract_planning::PRMConfigurator& rhs)
  {
    Node node;
    node["max_nearest_neighbors"] = rhs.max_nearest_neighbors;
    return node;
  }

  static bool decode(const Node& node, tesseract_planning::PRMConfigurator& rhs)
  {
    // Check for required entries
    if (const YAML::Node& n = node["max_nearest_neighbors"])
      rhs.max_nearest_neighbors = n.as<int>();
    return true;
  }
};

//=========================== PRMstarConfigurator ===========================
template <>
struct convert<tesseract_planning::PRMstarConfigurator>
{
  static Node encode(const tesseract_planning::PRMstarConfigurator& rhs)
  {
    Node node;
    return node;
  }

  static bool decode(const Node& node, tesseract_planning::PRMstarConfigurator& rhs)
  {
    // Check for required entries
    return true;
  }
};

//=========================== LazyPRMstarConfigurator ===========================
template <>
struct convert<tesseract_planning::LazyPRMstarConfigurator>
{
  static Node encode(const tesseract_planning::LazyPRMstarConfigurator& rhs)
  {
    Node node;
    return node;
  }

  static bool decode(const Node& node, tesseract_planning::LazyPRMstarConfigurator& rhs)
  {
    // Check for required entries
    return true;
  }
};

//=========================== SPARSConfigurator ===========================
template <>
struct convert<tesseract_planning::SPARSConfigurator>
{
  static Node encode(const tesseract_planning::SPARSConfigurator& rhs)
  {
    Node node;
    node["max_failures"] = rhs.max_failures;
    node["dense_delta_fraction"] = rhs.dense_delta_fraction;
    node["sparse_delta_fraction"] = rhs.sparse_delta_fraction;
    node["stretch_factor"] = rhs.stretch_factor;
    return node;
  }

  static bool decode(const Node& node, tesseract_planning::SPARSConfigurator& rhs)
  {
    // Check for required entries
    if (const YAML::Node& n = node["max_failures"])
      rhs.max_failures = n.as<int>();
    if (const YAML::Node& n = node["dense_delta_fraction"])
      rhs.dense_delta_fraction = n.as<double>();
    if (const YAML::Node& n = node["sparse_delta_fraction"])
      rhs.sparse_delta_fraction = n.as<double>();
    if (const YAML::Node& n = node["stretch_factor"])
      rhs.stretch_factor = n.as<double>();
    return true;
  }
};

//=========================== OMPLPlannerConfigurator ===========================
template <>
struct convert<std::vector<std::shared_ptr<const tesseract_planning::OMPLPlannerConfigurator>>>
{
  static Node encode(const std::vector<std::shared_ptr<const tesseract_planning::OMPLPlannerConfigurator>>& rhs)
  {
    Node node;
    for (const auto& ompl_configurator : rhs)
    {
      Node p_node;
      bool valid_type = true;
      tesseract_planning::OMPLPlannerType type = ompl_configurator->getType();

      switch (type)
      {
        case tesseract_planning::OMPLPlannerType::SBL:
        {
          auto planner = std::dynamic_pointer_cast<const tesseract_planning::SBLConfigurator>(ompl_configurator);
          p_node["SBLConfigurator"] = *planner;
          break;
        }
        case tesseract_planning::OMPLPlannerType::EST:
        {
          auto planner = std::dynamic_pointer_cast<const tesseract_planning::ESTConfigurator>(ompl_configurator);
          p_node["ESTConfigurator"] = *planner;
          break;
        }
        case tesseract_planning::OMPLPlannerType::LBKPIECE1:
        {
          auto planner = std::dynamic_pointer_cast<const tesseract_planning::LBKPIECE1Configurator>(ompl_configurator);
          p_node["LBKPIECE1Configurator"] = *planner;
          break;
        }
        case tesseract_planning::OMPLPlannerType::BKPIECE1:
        {
          auto planner = std::dynamic_pointer_cast<const tesseract_planning::BKPIECE1Configurator>(ompl_configurator);
          p_node["BKPIECE1Configurator"] = *planner;
          break;
        }
        case tesseract_planning::OMPLPlannerType::KPIECE1:
        {
          auto planner = std::dynamic_pointer_cast<const tesseract_planning::KPIECE1Configurator>(ompl_configurator);
          p_node["KPIECE1Configurator"] = *planner;
          break;
        }
        case tesseract_planning::OMPLPlannerType::BiTRRT:
        {
          auto planner = std::dynamic_pointer_cast<const tesseract_planning::BiTRRTConfigurator>(ompl_configurator);
          p_node["BiTRRTConfigurator"] = *planner;
          break;
        }
        case tesseract_planning::OMPLPlannerType::RRT:
        {
          auto planner = std::dynamic_pointer_cast<const tesseract_planning::RRTConfigurator>(ompl_configurator);
          p_node["RRTConfigurator"] = *planner;
          break;
        }
        case tesseract_planning::OMPLPlannerType::RRTConnect:
        {
          auto planner = std::dynamic_pointer_cast<const tesseract_planning::RRTConnectConfigurator>(ompl_configurator);
          p_node["RRTConnectConfigurator"] = *planner;
          break;
        }
        case tesseract_planning::OMPLPlannerType::RRTstar:
        {
          auto planner = std::dynamic_pointer_cast<const tesseract_planning::RRTstarConfigurator>(ompl_configurator);
          p_node["RRTstarConfigurator"] = *planner;
          break;
        }
        case tesseract_planning::OMPLPlannerType::TRRT:
        {
          auto planner = std::dynamic_pointer_cast<const tesseract_planning::TRRTConfigurator>(ompl_configurator);
          p_node["TRRTConfigurator"] = *planner;
          break;
        }
        case tesseract_planning::OMPLPlannerType::PRM:
        {
          auto planner = std::dynamic_pointer_cast<const tesseract_planning::PRMConfigurator>(ompl_configurator);
          p_node["PRMConfigurator"] = *planner;
          break;
        }
        case tesseract_planning::OMPLPlannerType::PRMstar:
        {
          auto planner = std::dynamic_pointer_cast<const tesseract_planning::PRMstarConfigurator>(ompl_configurator);
          p_node["PRMstarConfigurator"] = *planner;
          break;
        }
        case tesseract_planning::OMPLPlannerType::LazyPRMstar:
        {
          auto planner =
              std::dynamic_pointer_cast<const tesseract_planning::LazyPRMstarConfigurator>(ompl_configurator);
          p_node["LazyPRMstarConfigurator"] = *planner;
          break;
        }
        case tesseract_planning::OMPLPlannerType::SPARS:
        {
          auto planner = std::dynamic_pointer_cast<const tesseract_planning::SPARSConfigurator>(ompl_configurator);
          p_node["SPARSConfigurator"] = *planner;
          break;
        }
        default:
          valid_type = false;
          break;
      }
      node.push_back(p_node);
    }

    return node;
  }

  static bool decode(const Node& node,
                     std::vector<std::shared_ptr<const tesseract_planning::OMPLPlannerConfigurator>>& rhs)
  {
    // Check for required entries
    for (const auto& planner : node)
    {
      if (const YAML::Node& n = planner["SBLConfigurator"])
      {
        rhs.push_back(std::static_pointer_cast<const tesseract_planning::OMPLPlannerConfigurator>(
            std::make_shared<tesseract_planning::SBLConfigurator>(n.as<tesseract_planning::SBLConfigurator>())));
      }
      else if (const YAML::Node& n = planner["ESTConfigurator"])
      {
        rhs.push_back(std::static_pointer_cast<const tesseract_planning::OMPLPlannerConfigurator>(
            std::make_shared<tesseract_planning::ESTConfigurator>(n.as<tesseract_planning::ESTConfigurator>())));
      }
      else if (const YAML::Node& n = planner["LBKPIECE1Configurator"])
      {
        rhs.push_back(std::static_pointer_cast<const tesseract_planning::OMPLPlannerConfigurator>(
            std::make_shared<tesseract_planning::LBKPIECE1Configurator>(
                n.as<tesseract_planning::LBKPIECE1Configurator>())));
      }
      else if (const YAML::Node& n = planner["BKPIECE1Configurator"])
      {
        rhs.push_back(std::static_pointer_cast<const tesseract_planning::OMPLPlannerConfigurator>(
            std::make_shared<tesseract_planning::BKPIECE1Configurator>(
                n.as<tesseract_planning::BKPIECE1Configurator>())));
      }
      else if (const YAML::Node& n = planner["KPIECE1Configurator"])
      {
        rhs.push_back(std::static_pointer_cast<const tesseract_planning::OMPLPlannerConfigurator>(
            std::make_shared<tesseract_planning::KPIECE1Configurator>(
                n.as<tesseract_planning::KPIECE1Configurator>())));
      }
      else if (const YAML::Node& n = planner["BiTRRTConfigurator"])
      {
        rhs.push_back(std::static_pointer_cast<const tesseract_planning::OMPLPlannerConfigurator>(
            std::make_shared<tesseract_planning::BiTRRTConfigurator>(n.as<tesseract_planning::BiTRRTConfigurator>())));
      }
      else if (const YAML::Node& n = planner["RRTConfigurator"])
      {
        rhs.push_back(std::static_pointer_cast<const tesseract_planning::OMPLPlannerConfigurator>(
            std::make_shared<tesseract_planning::RRTConfigurator>(n.as<tesseract_planning::RRTConfigurator>())));
      }
      else if (const YAML::Node& n = planner["RRTConnectConfigurator"])
      {
        rhs.push_back(std::static_pointer_cast<const tesseract_planning::OMPLPlannerConfigurator>(
            std::make_shared<tesseract_planning::RRTConnectConfigurator>(
                n.as<tesseract_planning::RRTConnectConfigurator>())));
      }
      else if (const YAML::Node& n = planner["RRTstarConfigurator"])
      {
        rhs.push_back(std::static_pointer_cast<const tesseract_planning::OMPLPlannerConfigurator>(
            std::make_shared<tesseract_planning::RRTstarConfigurator>(
                n.as<tesseract_planning::RRTstarConfigurator>())));
      }
      else if (const YAML::Node& n = planner["TRRTConfigurator"])
      {
        rhs.push_back(std::static_pointer_cast<const tesseract_planning::OMPLPlannerConfigurator>(
            std::make_shared<tesseract_planning::TRRTConfigurator>(n.as<tesseract_planning::TRRTConfigurator>())));
      }
      else if (const YAML::Node& n = planner["PRMConfigurator"])
      {
        rhs.push_back(std::static_pointer_cast<const tesseract_planning::OMPLPlannerConfigurator>(
            std::make_shared<tesseract_planning::PRMConfigurator>(n.as<tesseract_planning::PRMConfigurator>())));
      }
      else if (const YAML::Node& n = planner["PRMstarConfigurator"])
      {
        rhs.push_back(std::static_pointer_cast<const tesseract_planning::OMPLPlannerConfigurator>(
            std::make_shared<tesseract_planning::PRMstarConfigurator>(
                n.as<tesseract_planning::PRMstarConfigurator>())));
      }
      else if (const YAML::Node& n = planner["LazyPRMstarConfigurator"])
      {
        rhs.push_back(std::static_pointer_cast<const tesseract_planning::OMPLPlannerConfigurator>(
            std::make_shared<tesseract_planning::LazyPRMstarConfigurator>(
                n.as<tesseract_planning::LazyPRMstarConfigurator>())));
      }
      else if (const YAML::Node& n = planner["SPARSConfigurator"])
      {
        rhs.push_back(std::static_pointer_cast<const tesseract_planning::OMPLPlannerConfigurator>(
            std::make_shared<tesseract_planning::SPARSConfigurator>(n.as<tesseract_planning::SPARSConfigurator>())));
      }
    }

    return true;
  }
};

//=========================== OMPLSolverConfig ===========================
template <>
struct convert<tesseract_planning::OMPLSolverConfig>
{
  static Node encode(const tesseract_planning::OMPLSolverConfig& rhs)
  {
    Node node;

    node["planning_time"] = rhs.planning_time;
    node["max_solutions"] = rhs.max_solutions;
    node["simplify"] = rhs.simplify;
    node["optimize"] = rhs.optimize;
    node["planners"] = rhs.planners;

    return node;
  }

  static bool decode(const Node& node, tesseract_planning::OMPLSolverConfig& rhs)
  {
    // Check for required entries
    if (const YAML::Node& n = node["planning_time"])
      rhs.planning_time = n.as<double>();
    if (const YAML::Node& n = node["max_solutions"])
      rhs.max_solutions = n.as<int>();
    if (const YAML::Node& n = node["simplify"])
      rhs.simplify = n.as<bool>();
    if (const YAML::Node& n = node["optimize"])
      rhs.optimize = n.as<bool>();
    if (const YAML::Node& n = node["planners"])
      rhs.planners = n.as<std::vector<std::shared_ptr<const tesseract_planning::OMPLPlannerConfigurator>>>();
    return true;
  }
};

}  // namespace YAML

#endif  // TESSERACT_MOTION_PLANNING_YAML_EXTENSIONS_H
