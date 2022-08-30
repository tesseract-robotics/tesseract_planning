^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_process_managers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.13.1 (2022-08-30)
-------------------
* Add back profile overrides to MoveInstruction
* Contributors: Levi Armstrong

0.13.0 (2022-08-25)
-------------------
* Add Eigen alignment macro to classes that contain SceneState
* Add ruckig trajectory smoothing
* Move most SWIG commands to tesseract_python package (`#227 <https://github.com/tesseract-robotics/tesseract_planning/issues/227>`_)
* Switch to using TypeErasure isNull
* Add appendInstruction to composite and remove push_back and insert methods
* Move isCompositeInstruction into InstructionPoly
* Remove NullWaypoint and NullInstruction
* Rename tesseract_command_language core directory to poly
* Rename Waypoint and Instruction to WaypointPoly and InstructionPoly
* Add CartesianWaypointPoly, JointWaypointPoly and StateWaypointPoly
* Refactor using MoveInstructionPoly
* Add TESSERACT_PROCESS_MANAGERS_HAS_TRAJOPT_IFOPT to tesseract_process_managers (`#225 <https://github.com/tesseract-robotics/tesseract_planning/issues/225>`_)
* Remove plan instruction
* Update code based on clang-tidy-14
* Add unit tests for fix_state_bounds_task_generator
* Contributors: John Wason, Levi Armstrong, Matthew Powelson

0.12.0 (2022-07-07)
-------------------
* Update ros_industrial_cmake_boilerplate to 0.3.0 (`#214 <https://github.com/tesseract-robotics/tesseract_planning/issues/214>`_)
* Move use of BOOST_CLASS_VERSION to header
* Add ability to submit ProcessPlanningProblem to planning server (`#213 <https://github.com/tesseract-robotics/tesseract_planning/issues/213>`_)
* Added CPack (`#208 <https://github.com/tesseract-robotics/tesseract_planning/issues/208>`_)
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Fix Typos
  - TrajOptMotionPlanner
  - DefaultProcessManagers
* Contributors: Levi Armstrong, Michael Ripperger, christian.petersmeier

0.11.0 (2022-06-20)
-------------------

0.10.4 (2022-06-03)
-------------------

0.10.3 (2022-05-31)
-------------------
* Switch test to use tcmalloc_minimal
* Add valid method to ProcessPlanningFuture
* Contributors: Levi Armstrong

0.10.2 (2022-05-24)
-------------------
* Make process planning server threadsafe and support multiple executors (`#197 <https://github.com/tesseract-robotics/tesseract_planning/issues/197>`_)
* Contributors: Levi Armstrong

0.10.1 (2022-05-09)
-------------------
* Add name to process planning problem
* Contributors: Levi Armstrong

0.10.0 (2022-05-03)
-------------------
* Remove duplicate serialization methods and use those in tesseract_common
* Store environment in process planning problem (`#192 <https://github.com/tesseract-robotics/tesseract_planning/issues/192>`_)
  * Store environment in process planning problem
  * Update readme dependency versions
* Add SWIG %shared_ptr to ProcessPlanningProblem (`#189 <https://github.com/tesseract-robotics/tesseract_planning/issues/189>`_)
* Contributors: John Wason, Levi Armstrong

0.9.9 (2022-04-22)
------------------
* Update ProcessPlanningFuture to leverage shared future (`#188 <https://github.com/tesseract-robotics/tesseract_planning/issues/188>`_)
  * Update ProcessPlanningFuture to leverage shared future
  * fix problem swid def
* Fix serialization for the ProcessPlanningFuture (`#187 <https://github.com/tesseract-robotics/tesseract_planning/issues/187>`_)
  * Change boost serialization tracking for ProcessPlanningRequest/Future
  * Add serialization for derived TaskInfos
  * Store pointer to environment in TaskInfo instead of a clone
* Contributors: Levi Armstrong, Matthew Powelson

0.9.8 (2022-04-19)
------------------

0.9.7 (2022-04-08)
------------------
* Fix logic in FixStateBounds for case ALL
* Contributors: Matthew Powelson

0.9.6 (2022-04-01)
------------------
* Fix issue in contact_check_profile not setting override type in default constructor
* Contributors: Levi Armstrong

0.9.5 (2022-03-31)
------------------
* Update to leverage TesseractSupportResourceLocator (`#181 <https://github.com/tesseract-robotics/tesseract_planning/issues/181>`_)
  * Update to leverage TesseractSupportResourceLocator
  * Update CI docker tag to 0.9
* Contributors: Levi Armstrong

0.9.4 (2022-03-25)
------------------
* Add serialization for ProcessPlanningRequest (`#174 <https://github.com/tesseract-robotics/tesseract_planning/issues/174>`_)
  * Switch serialization instantiations to use tesseract_common macro
  * Add serialization for ProcessPlanningRequest
  * Fix Windows build
* Add TESSERACT_ENABLE_EXAMPLES compile option (`#173 <https://github.com/tesseract-robotics/tesseract_planning/issues/173>`_)
* Contributors: John Wason, Matthew Powelson

0.9.3 (2022-02-22)
------------------
* Python patches for Feb 2022 update (`#172 <https://github.com/tesseract-robotics/tesseract_planning/issues/172>`_)
* Contributors: John Wason

0.9.2 (2022-02-07)
------------------

0.9.1 (2022-01-27)
------------------
* Update fix_state_collision_task to skip cartesian waypoints
* Contributors: Levi Armstrong

0.9.0 (2022-01-26)
------------------
* Fix thread safety issue with TaskInfoContainer
* Contributors: Levi Armstrong

0.8.1 (2022-01-24)
------------------

0.8.0 (2022-01-20)
------------------
* Expose save input and output for tasks in process planning request
* Update for fix in checkTrajectory and supporting functions
* Make ProcessPlanningServer::run const (`#160 <https://github.com/tesseract-robotics/tesseract_planning/issues/160>`_)
* Print collision details when fix state collision task trajopt failure
* Contributors: Levi Armstrong, Matthew Powelson

0.7.3 (2021-12-21)
------------------
* Add additional option to fix state collision profile
* Contributors: Levi Armstrong

0.7.2 (2021-12-16)
------------------
* Add upsample trajectory task generator
* Contributors: Levi Armstrong

0.7.1 (2021-12-15)
------------------
* Only check kinematics if built in debug (`#149 <https://github.com/tesseract-robotics/tesseract_planning/issues/149>`_)
  * Only check kinematics if built in debug
  * Global process plans should not fix raster start and end position based on the global results
  * Add typeid name to failed to find profile message
  * Fix clang-tidy issues
* Fix bug in getClosestJointSolution in simple planner utils
* Contributors: Levi Armstrong

0.7.0 (2021-12-06)
------------------
* Fix fix state collision task generator unit tests
* Update renaming of ContactManagerConfig variables
* Add ContactManagerConfig inside CollisionCheckConfig
* Correctly set collision margin data in collision checking utils
* Contributors: Levi Armstrong, Matthew Powelson

0.6.8 (2021-12-01)
------------------

0.6.7 (2021-11-30)
------------------

0.6.6 (2021-11-29)
------------------
* Fix ability to use same task with different parameters adding namespaces to the profile dictionary
* Update CI docker tag and target linking order (`#135 <https://github.com/tesseract-robotics/tesseract_planning/issues/135>`_)
  * Update CI docker tag
  * Update target linking order
* Contributors: Levi Armstrong

0.6.5 (2021-11-11 15:50)
------------------------

0.6.4 (2021-11-11 12:25)
------------------------
* Add lvs simple planner that does not use inverse kinematics and set as default
* Contributors: Levi-Armstrong

0.6.3 (2021-11-03)
------------------
* Update taskflows to leverage graph taskflow
* Move problem generator inside the motion planner
* Add profile dictionary to planning request
* Contributors: Levi-Armstrong

0.6.2 (2021-10-29)
------------------
* Make environment cache methods const (`#124 <https://github.com/tesseract-robotics/tesseract_planning/issues/124>`_)
  * Make environment cache methods const
  * Update docker version
* Contributors: Levi Armstrong

0.6.1 (2021-10-20)
------------------

0.6.0 (2021-10-13)
------------------
* Update tesseract_process_managers to leverage JointGroup and KinematicGroup
* Update due to changes related to trajopt
* Update debug_observer.cpp to leverage console bridge
* Update based on change in trajopt ifopt (`#90 <https://github.com/tesseract-robotics/tesseract_planning/issues/90>`_)
  Co-authored-by: cbw36 <cwolfe1996@gmail.com>
* Add trajectory container class to abstract command lanaguage from time parameterization (`#44 <https://github.com/tesseract-robotics/tesseract_planning/issues/44>`_)
* clang format and fixes for CI
* Add TaskInfo statistics and custom DOT file generator
* Add serialization for TaskInfo
  Changes after review and add instructions to the serialization unit test
* Add elapsed time to TaskInfo
* Set descrete lvs post check as default for taskflows
* Update to taskflow 3.0
* Fix passing of meta information in simple planner and min_seed_length
* Make Instruction and Waypoint default constructor private
* Switch type erasure cast methods to return references instead of pointer
* Rename Instruction and Waypoint cast and cast_const to as
* Remove NullWaypoint and NullInstruction types
* Reduce limits slightly in FixStateBoundsTaskGenerator
* Enable OMPL to plan for paths with more than one instruction (`#49 <https://github.com/tesseract-robotics/tesseract_planning/issues/49>`_)
  * Enable OMPL to plan for paths with more than one instruction
  * Rebase on latest and update unit test
  Co-authored-by: Levi Armstrong <levi.armstrong@swri.org>
* Run enforce bounds on result trajectory for all motion planners
* Correctly populate start instruction velocities in TOTG
* Fix ProfileDictionary use and profile entries in Python
* FixStateCollision: Only adjust the states that are in collision
* Update to use boost targets (`#46 <https://github.com/tesseract-robotics/tesseract_planning/issues/46>`_)
* Switch to using Eigen target
* Add profile overrides to Move, Plan, and Composite Instructions
* Fix passing of meta information through TOTG
  Note that it will still be partially lost if it change in the middle of a sub-composite.
* Add sub-composite rescaling to TOTG task generator
* Allow nonconditional nodes to be attached to arbitrary nodes
* Allow graph taskflows that connect non-leaf nodes to error/done
* Fix misc typos
* Add boost serialization for the command language along with unit tests
* Add the ability to save the Task inputs and outputs to the TaskInfo
* Update to latest tesseract_environment changes
* Remove tcmalloc as a dependency in package.xml and cmake config.in
* Fix bug in task info
  The name was getting stored in message instead of task_name
* Update cmake_common_scripts to ros_industrial_cmake_boilerplate
* Remove tcmalloc as a library dependency. tcmalloc should only be used when liking an executable.
* Add time optimal trajectory generation TOTG (`#23 <https://github.com/tesseract-robotics/tesseract_planning/issues/23>`_)
* Switch tesseract descartes to use float by default
* Update due to changes in tesseract_visualization interface
* Add google tcmalloc to planning server (`#19 <https://github.com/tesseract-robotics/tesseract_planning/issues/19>`_)
  * Add google tcmalloc to planning server
  * Add memory usage example
  * Fix focal ci by adding ici_with_unset_variables EMBED script
  * Update readme to include tcmalloc and remove glibc documentation
  * Add tcmalloc to windows ci
  * Disable using tcmalloc for windows builds
  * Disable memory usage example for windows build
* Graph Taskflow Generator Revision (`#17 <https://github.com/tesseract-robotics/tesseract_planning/issues/17>`_)
* Improve simple planner profiles to handle working frame
* Update packages package.xml to include buildtool_depend on cmake and exec_depend on catkin
* Switch DebugObserver to use console bridge
* Add error task and done task to GraphTaskflow
* Move ProcessInfo into ProcessInterface for outside access (`#514 <https://github.com/tesseract-robotics/tesseract_planning/issues/514>`_)
  * Move ProcessInfo into ProcessInterface for outside access
  * Rename Process to Task for generators and associated types
  ProcessGenerator -> TaskGenerator
  ProcessInterface -> TaskflowInterface
  ProcessInfo -> TaskInfo
  ProcessInfoContainer -> TaskInfoContainer
  ProcessInput -> TaskInput
  * Fix remaining changes
  Co-authored-by: Levi Armstrong <levi.armstrong@swri.org>
* Update motion planners to account for Joint and State Waypoints unordered joints relative to kinematics
* Utilize  parameter in TrajOpt planner
* Update to use initialize_code_coverage() macro and compiler definition
* Extract package name and version from package.xml
* Remove process_managers, replaced by planning server
* Python package updates for command language
* Simplify the process generator interface to avoid std::function
* Make changes to better support python wrapping
* Remove tesseract package
* Add TrajOpt Solver Profile
* Add core directory to tesseract_process_managers
* Switch to using lambda over std::bind and remove NOLINT
* Add bool has_seed to ProcessInput and add back GraphTaskflow
* More documentation, remove commented code, some requested changes
* Add doxygen and a few bug fixes
* Make profiles and ProfileDictionary const
* Restructure taskflow generators to support composition
* Make trajopt, ompl, descartes, freespace and cartesian taskflow generators
* Fix enabling of simple planner
* Move default process planners to method that user calls
* Add profile dictionary
* Create process planning server
* Add feedback of contacts to FixStateCollisionProcessGenerator
* Add virtual destructor to ProcessInfo as well as bug fixes
* Add ProcessInfo to process generators (`#450 <https://github.com/tesseract-robotics/tesseract_planning/issues/450>`_)
* Add CollisionCheckConfig
* Fix bug in simple planner not resetting start waypoint
* Code Simplification in StateInCollision
* Move ManipulatorManager into Environment
* Add MoveWaypointFromCollisionRandomSampler to FixStateCollisionProcess (`#426 <https://github.com/tesseract-robotics/tesseract_planning/issues/426>`_)
  * Add MoveWaypointFromCollisionRandomSampler to FixStateCollisionProcess
  * Add more generalized way of specifying correction methods
  * Bug fix
  * Add assert to catch mismatched sizes
  * Rebase fixes and a bug fix
* Add seed min length process generator and unit tests
* Update unit tests and fix lvs_interpolation
* Add verbose options to process input so planner verbosity can be turned on
* Fix issue in freespace taskflow for the trajopt first condition
* Add raster only process managers
* Add ProfileSwitchProcessGenerator
  This generator simply returns a value specified in the composite profile. This can be used to switch execution based on the profile
* Add utility for getting profiles (`#412 <https://github.com/tesseract-robotics/tesseract_planning/issues/412>`_)
* Enable tesseract_motion_planners build on windows
* Address console bridge issue `#91 <https://github.com/tesseract-robotics/tesseract_planning/issues/91>`_
* Fix to handle console_bridge target renaming in noetic
* Separate public and private compiler option and add back -mno-avx
* Add individual CI badges and Windows CI build
* Check validity of longest valid segment
* Set active links based on ManipulatorInfo in contact check processes
* Add visibility control to all packages
* Update due to changes in descartes compound edge evaluator
* Fix done and error callback in simple process manager
* Improve global raster taskflow
* Update default longest valid segment length
* Add taskflow debug and profile observer
* Change freespace taskflow to still try trajopt if ompl fails
* Fix graph taskflow handeling of TASK type
* Change ProcessInput to better support changing data structure throughout the taskflow
* Update REP and ROP Kinematics along with ManipulatorInfo TCP support
* Add global raster variant
* Improve ignition material conversion
* Add manipulator manager to support tracking changes in the future
* Refactor fix state bounds utils to eliminate repetitive inform msgs
* Pass verbose to motion planners only when debug messages enabled
* Leverage cmake_common_scripts
* Add fixStateBoundsProcessGenerator
* Clean up tesseract_process_managers and tesseract_motion_planners package
* Add fixStateBoundsProcessGenerator
* Switch ISP to use MoveInstructions instead of PlanInstructions
* Add Profiles to ISP Time parameterization process generator
* Process managers: Only print "Generating Taskflow for..." when log debug
* Split command_language_utils into multiple files
* Simplify raster example program
* Add simple process manager and planner profile mapping
* Expose velocity and acceleration scaling factors in process generators
* Add debugging information when planning fails due to collisions
* Fix typo in ISP ProcessGenerator
* Fix Clang Tidy errors
* Fix/Add clearing of graph and sequential taskflow
* Add graph taskflow
* Add iterative spline parameterization process generator
* Remove random generators and validators
* Add discrete and continuous process generators
* Switch to using unique pointer for Process Generator
* Rename sequential_failure_tree_taskflow to sequential_taskflow
* Make command language utility function generic and move planner specific ones to motion planners package
* Get tesseract process managers working
* Swith process input to leverage pointer instead of references
* Update tesseract_process_managers
* Update/Fix tesseract process manager
* Add unit tests for fixed size assign position
* Add missing include <atomic>
* Bring back generateSeed, add readme, and add task validators
* Add ManipulatorInfo to PlanInstruction
* Misc improvements and rebase fixes
  Modify examples so the complete successfully and clean some things
* Update Defaults and add ability to abort process
* Add OMPL and Descartes support
* Update start and end Instructions in process managers
* tesseract_process_managers: Add raster_process_manager
  Adds the groundwork for a raster process manager along with an example using random processes.
* tesseract_process_managers initial commit
* Contributors: John Wason, Levi Armstrong, Levi-Armstrong, Matthew Powelson, Michael Ripperger, Tyler Marr
