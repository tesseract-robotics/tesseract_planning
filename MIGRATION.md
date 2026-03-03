# Migration Guide: tesseract_planning Consolidation

This guide helps downstream users migrate from the old multi-package `tesseract_planning` layout to the new unified single-package layout.

## Summary of Changes

- All `tesseract_*` subdirectories renamed (dropping the prefix)
- Include paths changed from `tesseract_X/` to `tesseract/X/`
- Core headers moved up one level (e.g., `tesseract/motion_planners/core/*.h` → `tesseract/motion_planners/*.h`)
- All sub-packages consolidated into a single CMake project (`tesseract_planning`)
- CMake targets renamed with `tesseract::` ALIAS targets
- Single `package.xml` for the entire project

## Directory Mapping

| Old                              | New                      |
|----------------------------------|--------------------------|
| `tesseract_command_language/`    | `command_language/`      |
| `tesseract_examples/`           | `examples/`              |
| `tesseract_motion_planners/`    | `motion_planners/`       |
| `tesseract_task_composer/`      | `task_composer/`         |
| `tesseract_time_parameterization/` | `time_parameterization/` |

## Include Path Mapping

| Old                                              | New                                           |
|--------------------------------------------------|-----------------------------------------------|
| `tesseract_command_language/*.h`                 | `tesseract/command_language/*.h`              |
| `tesseract_command_language/poly/*.h`            | `tesseract/command_language/poly/*.h`         |
| `tesseract_examples/*.h`                         | `tesseract/examples/*.h`                      |
| `tesseract_motion_planners/core/*.h`             | `tesseract/motion_planners/*.h`               |
| `tesseract_motion_planners/simple/*.h`           | `tesseract/motion_planners/simple/*.h`        |
| `tesseract_motion_planners/ompl/*.h`             | `tesseract/motion_planners/ompl/*.h`          |
| `tesseract_motion_planners/descartes/*.h`        | `tesseract/motion_planners/descartes/*.h`     |
| `tesseract_motion_planners/trajopt/*.h`          | `tesseract/motion_planners/trajopt/*.h`       |
| `tesseract_motion_planners/trajopt_ifopt/*.h`    | `tesseract/motion_planners/trajopt_ifopt/*.h` |
| `tesseract_task_composer/core/*.h`               | `tesseract/task_composer/*.h`                 |
| `tesseract_task_composer/core/nodes/*.h`         | `tesseract/task_composer/nodes/*.h`           |
| `tesseract_task_composer/core/test_suite/*.h`    | `tesseract/task_composer/test_suite/*.h`      |
| `tesseract_task_composer/planning/*.h`           | `tesseract/task_composer/planning/*.h`        |
| `tesseract_task_composer/taskflow/*.h`           | `tesseract/task_composer/taskflow/*.h`        |
| `tesseract_time_parameterization/core/*.h`       | `tesseract/time_parameterization/*.h`         |
| `tesseract_time_parameterization/isp/*.h`        | `tesseract/time_parameterization/isp/*.h`     |
| `tesseract_time_parameterization/kdl/*.h`        | `tesseract/time_parameterization/kdl/*.h`     |
| `tesseract_time_parameterization/totg/*.h`       | `tesseract/time_parameterization/totg/*.h`    |
| `tesseract_time_parameterization/ruckig/*.h`     | `tesseract/time_parameterization/ruckig/*.h`  |

## CMake Target Mapping

| Old Target                                            | New Target                                   |
|-------------------------------------------------------|----------------------------------------------|
| `tesseract::tesseract_command_language`               | `tesseract::command_language`                |
| `tesseract::tesseract_motion_planners_core`           | `tesseract::motion_planners`                 |
| `tesseract::tesseract_motion_planners`                | `tesseract::motion_planners`                 |
| `tesseract::tesseract_motion_planners_simple`         | `tesseract::motion_planners_simple`          |
| `tesseract::tesseract_motion_planners_ompl`           | `tesseract::motion_planners_ompl`            |
| `tesseract::tesseract_motion_planners_descartes`      | `tesseract::motion_planners_descartes`       |
| `tesseract::tesseract_motion_planners_trajopt`        | `tesseract::motion_planners_trajopt`         |
| `tesseract::tesseract_motion_planners_trajopt_ifopt`  | `tesseract::motion_planners_trajopt_ifopt`   |
| `tesseract::tesseract_task_composer`                  | `tesseract::task_composer`                   |
| `tesseract::tesseract_task_composer_nodes`            | `tesseract::task_composer_nodes`             |
| `tesseract::tesseract_task_composer_planning_nodes`   | `tesseract::task_composer_planning_nodes`    |
| `tesseract::tesseract_task_composer_taskflow`         | `tesseract::task_composer_taskflow`          |
| `tesseract::tesseract_time_parameterization_core`     | `tesseract::time_parameterization`           |
| `tesseract::tesseract_time_parameterization`          | `tesseract::time_parameterization`           |
| `tesseract::tesseract_time_parameterization_isp`      | `tesseract::time_parameterization_isp`       |
| `tesseract::tesseract_time_parameterization_kdl`      | `tesseract::time_parameterization_kdl`       |
| `tesseract::tesseract_time_parameterization_totg`     | `tesseract::time_parameterization_totg`      |
| `tesseract::tesseract_time_parameterization_ruckig`   | `tesseract::time_parameterization_ruckig`    |
| `tesseract::tesseract_examples`                       | `tesseract::examples`                        |

## find_package Mapping

| Old                                                                                   | New                                                                                                  |
|---------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------|
| `find_package(tesseract_command_language REQUIRED)`                                   | `find_package(tesseract_planning REQUIRED COMPONENTS command_language)`                              |
| `find_package(tesseract_motion_planners REQUIRED COMPONENTS core simple)`             | `find_package(tesseract_planning REQUIRED COMPONENTS motion_planners motion_planners_simple)`   |
| `find_package(tesseract_motion_planners REQUIRED COMPONENTS core OPTIONAL_COMPONENTS ompl)` | `find_package(tesseract_planning REQUIRED COMPONENTS motion_planners OPTIONAL_COMPONENTS motion_planners_ompl)` |
| `find_package(tesseract_task_composer REQUIRED)`                                      | `find_package(tesseract_planning REQUIRED COMPONENTS task_composer)`                            |
| `find_package(tesseract_task_composer REQUIRED COMPONENTS core planning taskflow)`    | `find_package(tesseract_planning REQUIRED COMPONENTS task_composer task_composer_planning task_composer_taskflow)` |
| `find_package(tesseract_time_parameterization REQUIRED COMPONENTS core isp totg)`     | `find_package(tesseract_planning REQUIRED COMPONENTS time_parameterization time_parameterization_isp time_parameterization_totg)` |
| `find_package(tesseract_examples REQUIRED)`                                           | `find_package(tesseract_planning REQUIRED COMPONENTS examples)`                                     |

## Migration Script

Run this script from the root of your downstream project to update all references:

```bash
#!/bin/bash
set -euo pipefail

echo "=== Migrating includes ==="
find . -type f \( -name '*.h' -o -name '*.hpp' -o -name '*.cpp' \) -exec sed -i \
  -e 's|#include <tesseract_command_language/|#include <tesseract/command_language/|g' \
  -e 's|#include <tesseract_motion_planners/core/|#include <tesseract/motion_planners/|g' \
  -e 's|#include <tesseract_motion_planners/|#include <tesseract/motion_planners/|g' \
  -e 's|#include <tesseract_task_composer/core/|#include <tesseract/task_composer/|g' \
  -e 's|#include <tesseract_task_composer/|#include <tesseract/task_composer/|g' \
  -e 's|#include <tesseract_time_parameterization/core/|#include <tesseract/time_parameterization/|g' \
  -e 's|#include <tesseract_time_parameterization/|#include <tesseract/time_parameterization/|g' \
  -e 's|#include <tesseract_examples/|#include <tesseract/examples/|g' \
  {} +

echo "=== Migrating CMake targets ==="
find . -name 'CMakeLists.txt' -exec sed -i \
  -e 's|tesseract::tesseract_command_language|tesseract::command_language|g' \
  -e 's|tesseract::tesseract_motion_planners_core|tesseract::motion_planners|g' \
  -e 's|tesseract::tesseract_motion_planners_trajopt_ifopt|tesseract::motion_planners_trajopt_ifopt|g' \
  -e 's|tesseract::tesseract_motion_planners_trajopt|tesseract::motion_planners_trajopt|g' \
  -e 's|tesseract::tesseract_motion_planners_simple|tesseract::motion_planners_simple|g' \
  -e 's|tesseract::tesseract_motion_planners_ompl|tesseract::motion_planners_ompl|g' \
  -e 's|tesseract::tesseract_motion_planners_descartes|tesseract::motion_planners_descartes|g' \
  -e 's|tesseract::tesseract_motion_planners|tesseract::motion_planners|g' \
  -e 's|tesseract::tesseract_task_composer_planning_nodes|tesseract::task_composer_planning_nodes|g' \
  -e 's|tesseract::tesseract_task_composer_taskflow|tesseract::task_composer_taskflow|g' \
  -e 's|tesseract::tesseract_task_composer_nodes|tesseract::task_composer_nodes|g' \
  -e 's|tesseract::tesseract_task_composer|tesseract::task_composer|g' \
  -e 's|tesseract::tesseract_time_parameterization_core|tesseract::time_parameterization|g' \
  -e 's|tesseract::tesseract_time_parameterization_isp|tesseract::time_parameterization_isp|g' \
  -e 's|tesseract::tesseract_time_parameterization_kdl|tesseract::time_parameterization_kdl|g' \
  -e 's|tesseract::tesseract_time_parameterization_totg|tesseract::time_parameterization_totg|g' \
  -e 's|tesseract::tesseract_time_parameterization_ruckig|tesseract::time_parameterization_ruckig|g' \
  -e 's|tesseract::tesseract_time_parameterization|tesseract::time_parameterization|g' \
  -e 's|tesseract::tesseract_examples|tesseract::examples|g' \
  {} +

echo "=== Migrating find_package calls ==="
# Note: find_package updates require manual review since component names changed.
# The following handles common patterns:
find . -name 'CMakeLists.txt' -exec sed -i \
  -e 's|find_package(tesseract_command_language REQUIRED)|find_package(tesseract_planning REQUIRED COMPONENTS command_language)|g' \
  -e 's|find_package(tesseract_examples REQUIRED)|find_package(tesseract_planning REQUIRED COMPONENTS examples)|g' \
  {} +

echo "=== Done. Review find_package calls for tesseract_motion_planners, tesseract_task_composer, and tesseract_time_parameterization manually. ==="
echo "These had component-based find_package calls that need component name updates (e.g., core -> motion_planners)."
```

> **Note:** The `find_package` migration for component-based packages (`tesseract_motion_planners`, `tesseract_task_composer`, `tesseract_time_parameterization`) requires manual attention since component names now include the package prefix (e.g., `core` → `motion_planners`).
