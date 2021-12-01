^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_process_managers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

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
