find_package(tesseract REQUIRED COMPONENTS kinematics environment)

# Core
list(APPEND SUPPORTED_COMPONENTS time_parameterization)
add_subdirectory(core)

# ISP
option(TESSERACT_BUILD_ISP "Build the Iterative Spline Parameterization (ISP) time parameterization implementation" ON)
if(TESSERACT_BUILD_ISP)
  message("Building ISP")
  list(APPEND SUPPORTED_COMPONENTS time_parameterization_isp)
  add_subdirectory(isp)
endif()

# KDL
option(TESSERACT_BUILD_KDL "Build the Constant TCP Speed Parameterization (KDL) time parameterization implementation"
       ON)
if(TESSERACT_BUILD_KDL)
  message("Building KDL")
  list(APPEND SUPPORTED_COMPONENTS time_parameterization_kdl)
  add_subdirectory(kdl)
endif()

# TOTG
option(TESSERACT_BUILD_TOTG "Build the Time Optimal Trajectory Generation (TOTG) time parameterization implementation"
       ON)
if(TESSERACT_BUILD_TOTG)
  message("Building TOTG")
  list(APPEND SUPPORTED_COMPONENTS time_parameterization_totg)
  add_subdirectory(totg)
endif()

# RUCKIG
option(TESSERACT_BUILD_RUCKIG "Build the Ruckig Trajectory Smoothing time parameterization implementation" ON)
if(TESSERACT_BUILD_RUCKIG)
  message("Building Ruckig")
  list(APPEND SUPPORTED_COMPONENTS time_parameterization_ruckig)
  add_subdirectory(ruckig)
endif()

set(SUPPORTED_COMPONENTS ${SUPPORTED_COMPONENTS} PARENT_SCOPE)
