^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_time_parameterization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.21.5 (2023-12-13)
-------------------

0.21.4 (2023-11-21)
-------------------

0.21.3 (2023-11-20)
-------------------

0.21.2 (2023-11-17)
-------------------

0.21.1 (2023-11-17)
-------------------

0.21.0 (2023-11-10)
-------------------
* Unused includes cleanup
* Contributors: Roelof Oomen

0.20.1 (2023-10-02)
-------------------

0.20.0 (2023-09-29)
-------------------

0.19.0 (2023-09-05)
-------------------
* Update to leverage cmake components
* Fixed issue where tesseract_common_trajectory.cpp wasn't getting built (`#356 <https://github.com/tesseract-robotics/tesseract_planning/issues/356>`_)
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Contributors: Levi Armstrong, Tyler Marr

0.18.4 (2023-07-07)
-------------------

0.18.3 (2023-07-04)
-------------------

0.18.2 (2023-07-03)
-------------------

0.18.1 (2023-07-03)
-------------------

0.18.0 (2023-06-30)
-------------------

0.17.0 (2023-06-06)
-------------------
* Fix incorrect include path in tesseract_common_trajectory.h
* Contributors: Levi Armstrong

0.16.3 (2023-05-03)
-------------------

0.16.2 (2023-04-28)
-------------------

0.16.1 (2023-04-11)
-------------------

0.16.0 (2023-04-09)
-------------------
* Update to support new contact results class (`#297 <https://github.com/tesseract-robotics/tesseract_planning/issues/297>`_)
* Contributors: Levi Armstrong

0.15.5 (2023-03-22)
-------------------
* Fix numerical issue in totg
* Fix TOTG assignData
* Contributors: Levi Armstrong

0.15.4 (2023-03-16)
-------------------

0.15.3 (2023-03-15)
-------------------

0.15.2 (2023-03-14)
-------------------

0.15.1 (2023-03-09)
-------------------

0.15.0 (2023-03-03)
-------------------
* Merge pull request `#269 <https://github.com/tesseract-robotics/tesseract_planning/issues/269>`_ from marip8/update/time-param-org
  Added optional builds of time parameterization implementations
* Created separate targets for each time parameterization implementation
* Moved core into own subdirectory; moved headers into specific sub-directories; updated unit tests
* Update tesseract_time_parameterization-config.cmake.in
* Added optional builds of time parameterization implementations
* Remove composite start instruction
* Contributors: Levi Armstrong, Michael Ripperger

0.14.0 (2022-10-23)
-------------------

0.13.1 (2022-08-30)
-------------------

0.13.0 (2022-08-25)
-------------------
* Add ruckig trajectory smoothing
* Move most SWIG commands to tesseract_python package (`#227 <https://github.com/tesseract-robotics/tesseract_planning/issues/227>`_)
* Remove NullWaypoint and NullInstruction
* Rename tesseract_command_language core directory to poly
* Rename Waypoint and Instruction to WaypointPoly and InstructionPoly
* Add CartesianWaypointPoly, JointWaypointPoly and StateWaypointPoly
* Refactor using MoveInstructionPoly
* Update code based on clang-tidy-14
* Contributors: John Wason, Levi Armstrong

0.12.0 (2022-07-07)
-------------------
* Update ros_industrial_cmake_boilerplate to 0.3.0 (`#214 <https://github.com/tesseract-robotics/tesseract_planning/issues/214>`_)
* Added CPack (`#208 <https://github.com/tesseract-robotics/tesseract_planning/issues/208>`_)
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Add support for sco::Optimizer::Callbacks to the trajopt solver profile (`#207 <https://github.com/tesseract-robotics/tesseract_planning/issues/207>`_)
* Contributors: Levi Armstrong, Michael Ripperger

0.11.0 (2022-06-20)
-------------------

0.10.4 (2022-06-03)
-------------------

0.10.3 (2022-05-31)
-------------------

0.10.2 (2022-05-24)
-------------------

0.10.1 (2022-05-09)
-------------------

0.10.0 (2022-05-03)
-------------------

0.9.9 (2022-04-22)
------------------

0.9.8 (2022-04-19)
------------------

0.9.7 (2022-04-08)
------------------

0.9.6 (2022-04-01)
------------------

0.9.5 (2022-03-31)
------------------
* Add check for time increasing in TOTG (`#175 <https://github.com/tesseract-robotics/tesseract_planning/issues/175>`_)
* Contributors: Levi Armstrong

0.9.4 (2022-03-25)
------------------

0.9.3 (2022-02-22)
------------------

0.9.2 (2022-02-07)
------------------
* Add robust method for assigning data for TOTG (`#169 <https://github.com/tesseract-robotics/tesseract_planning/issues/169>`_)
* Contributors: Levi Armstrong

0.9.1 (2022-01-27)
------------------

0.9.0 (2022-01-26)
------------------

0.8.1 (2022-01-24)
------------------

0.8.0 (2022-01-20)
------------------

0.7.3 (2021-12-21)
------------------

0.7.2 (2021-12-16)
------------------

0.7.1 (2021-12-15)
------------------
* Only check kinematics if built in debug (`#149 <https://github.com/tesseract-robotics/tesseract_planning/issues/149>`_)
  * Only check kinematics if built in debug
  * Global process plans should not fix raster start and end position based on the global results
  * Add typeid name to failed to find profile message
  * Fix clang-tidy issues
* Contributors: Levi Armstrong

0.7.0 (2021-12-06)
------------------

0.6.8 (2021-12-01)
------------------

0.6.7 (2021-11-30)
------------------

0.6.6 (2021-11-29)
------------------
* Update CI docker tag and target linking order (`#135 <https://github.com/tesseract-robotics/tesseract_planning/issues/135>`_)
  * Update CI docker tag
  * Update target linking order
* Contributors: Levi Armstrong

0.6.5 (2021-11-11 15:50)
------------------------

0.6.4 (2021-11-11 12:25)
------------------------

0.6.3 (2021-11-03)
------------------

0.6.2 (2021-10-29)
------------------

0.6.1 (2021-10-20)
------------------

0.6.0 (2021-10-13)
------------------
* Update based on change in trajopt ifopt (`#90 <https://github.com/tesseract-robotics/tesseract_planning/issues/90>`_)
  Co-authored-by: cbw36 <cwolfe1996@gmail.com>
* Add SWIG shared_ptr to trajectory containers
* Add trajectory container class to abstract command lanaguage from time parameterization (`#44 <https://github.com/tesseract-robotics/tesseract_planning/issues/44>`_)
* Add missing console bridge header to iterative spline test
* Make Instruction and Waypoint default constructor private
* Switch type erasure cast methods to return references instead of pointer
* Rename Instruction and Waypoint cast and cast_const to as
* Remove NullWaypoint and NullInstruction types
* Correctly populate start instruction velocities in TOTG
* Fix ProfileDictionary use and profile entries in Python
* Update to use boost targets (`#46 <https://github.com/tesseract-robotics/tesseract_planning/issues/46>`_)
* Switch to using Eigen target
* Fix passing of meta information through TOTG
  Note that it will still be partially lost if it change in the middle of a sub-composite.
* Add sub-composite rescaling to TOTG task generator
* Update to latest tesseract_environment changes
* Fix divide by zero case in time optimal trajectory generation
* Fix unitialized data in time optimal time parameterization and start to fixing float equal comparison
* Add workaround for TOTG failure on duplicate points
* Update cmake_common_scripts to ros_industrial_cmake_boilerplate
* Add time optimal trajectory generation TOTG (`#23 <https://github.com/tesseract-robotics/tesseract_planning/issues/23>`_)
* Update packages package.xml to include buildtool_depend on cmake and exec_depend on catkin
* Update to use initialize_code_coverage() macro and compiler definition
* Extract package name and version from package.xml
* Python package updates for command language
* Add missing colcon.pkg files
* Address console bridge issue `#91 <https://github.com/tesseract-robotics/tesseract_planning/issues/91>`_
* Fix to handle console_bridge target renaming in noetic
* Separate public and private compiler option and add back -mno-avx
* Add individual CI badges and Windows CI build
* Add visibility control to all packages
* Leverage cmake_common_scripts
* Allow ISP scaling factors to be changed on a point by point basis
* Split command_language_utils into multiple files
* Fix Clang Tidy errors
* Rename iterative spline parameterization methods
* Add tesseract_time_parameterization package include iterative spline algorithm
* Contributors: John Wason, Levi Armstrong, Matthew Powelson
