find_package(Threads REQUIRED)
find_package(tesseract REQUIRED COMPONENTS kinematics environment)

# Core
list(APPEND SUPPORTED_COMPONENTS motion_planners)
add_subdirectory(core)

# Simple
list(APPEND SUPPORTED_COMPONENTS motion_planners_simple)
add_subdirectory(simple)

# OMPL
option(TESSERACT_BUILD_OMPL "Build the OMPL planner" ON)
if(TESSERACT_BUILD_OMPL)
  message("Building OMPL Planner")
  list(APPEND SUPPORTED_COMPONENTS motion_planners_ompl)
  add_subdirectory(ompl)
endif()

# Descartes
option(TESSERACT_BUILD_DESCARTES "Build the Descartes planner" ON)
if(TESSERACT_BUILD_DESCARTES)
  message("Building Descartes Planner")
  list(APPEND SUPPORTED_COMPONENTS motion_planners_descartes)
  add_subdirectory(descartes)
endif()

# Trajopt
option(TESSERACT_BUILD_TRAJOPT "Build the Trajopt planner" ON)
if(TESSERACT_BUILD_TRAJOPT)
  message("Building Trajopt Planner")
  list(APPEND SUPPORTED_COMPONENTS motion_planners_trajopt)
  add_subdirectory(trajopt)
endif()

# Trajopt IFOPT
option(TESSERACT_BUILD_TRAJOPT_IFOPT "Build the Trajopt IFOPT planner" ON)
if(TESSERACT_BUILD_TRAJOPT_IFOPT)
  message("Building Trajopt IFOPT Planner")
  list(APPEND SUPPORTED_COMPONENTS motion_planners_trajopt_ifopt)
  add_subdirectory(trajopt_ifopt)
endif()

# Examples
if(TESSERACT_ENABLE_EXAMPLES)
  add_subdirectory(examples)
endif()

set(SUPPORTED_COMPONENTS ${SUPPORTED_COMPONENTS} PARENT_SCOPE)
