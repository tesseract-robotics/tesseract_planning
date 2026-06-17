^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_planning
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Unreleased
----------
* Add ``simplify_time`` to ``OMPLSolverConfig`` to time-bound OMPL path simplification (``simplifySolution(double)``); defaults to 0.0 which preserves the existing unbounded ``simplifyMax`` behavior

0.35.0 (2026-05-28)
-------------------
* Update descartes tag
* Export task_composer_planning_factories anchor
* Update conda build (`#746 <https://github.com/tesseract-robotics/tesseract_planning/issues/746>`_)
* Fix missing fstream include
* Add trajopt_ifopt support for Windows and MacOS (`#584 <https://github.com/tesseract-robotics/tesseract_planning/issues/584>`_)
* Mac Build Fixes (`#744 <https://github.com/tesseract-robotics/tesseract_planning/issues/744>`_)
* Bump johnwason/vcpkg-action from 7 to 8 (`#743 <https://github.com/tesseract-robotics/tesseract_planning/issues/743>`_)
  Bumps [johnwason/vcpkg-action](https://github.com/johnwason/vcpkg-action) from 7 to 8.
  - [Release notes](https://github.com/johnwason/vcpkg-action/releases)
  - [Commits](https://github.com/johnwason/vcpkg-action/compare/v7...v8)
  ---
  updated-dependencies:
  - dependency-name: johnwason/vcpkg-action
  dependency-version: '8'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
  Co-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>
* Windows Build Fixes (`#742 <https://github.com/tesseract-robotics/tesseract_planning/issues/742>`_)
* Fix formatJointPosition & FormatAsInputTask aux-vector reorder bugs (`#739 <https://github.com/tesseract-robotics/tesseract_planning/issues/739>`_)
* Update pull request paths
* Add environment/cereal_serialization.h include to task_composer/planning/cereal_serialization.h
* Update test CMakeLists to use find_gtest from RICB
* Fix CI filtering
* Bump docker/build-push-action from 6 to 7
  Bumps [docker/build-push-action](https://github.com/docker/build-push-action) from 6 to 7.
  - [Release notes](https://github.com/docker/build-push-action/releases)
  - [Commits](https://github.com/docker/build-push-action/compare/v6...v7)
  ---
  updated-dependencies:
  - dependency-name: docker/build-push-action
  dependency-version: '7'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
* Bump docker/metadata-action from 5 to 6
  Bumps [docker/metadata-action](https://github.com/docker/metadata-action) from 5 to 6.
  - [Release notes](https://github.com/docker/metadata-action/releases)
  - [Commits](https://github.com/docker/metadata-action/compare/v5...v6)
  ---
  updated-dependencies:
  - dependency-name: docker/metadata-action
  dependency-version: '6'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
* Bump docker/login-action from 3 to 4
  Bumps [docker/login-action](https://github.com/docker/login-action) from 3 to 4.
  - [Release notes](https://github.com/docker/login-action/releases)
  - [Commits](https://github.com/docker/login-action/compare/v3...v4)
  ---
  updated-dependencies:
  - dependency-name: docker/login-action
  dependency-version: '4'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
* Update to be single cmake project
* Bump actions/upload-artifact from 6 to 7
  Bumps [actions/upload-artifact](https://github.com/actions/upload-artifact) from 6 to 7.
  - [Release notes](https://github.com/actions/upload-artifact/releases)
  - [Commits](https://github.com/actions/upload-artifact/compare/v6...v7)
  ---
  updated-dependencies:
  - dependency-name: actions/upload-artifact
  dependency-version: '7'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
* Improve error output and messaging for solver status of TrajoptIfopt (`#723 <https://github.com/tesseract-robotics/tesseract_planning/issues/723>`_)
* Tesseract single cmake project consolidation
* Update to clang-format-18
* Leverage nested namespaces
* Contributors: John Wason, Levi Armstrong, Roelof Oomen, dependabot[bot]

0.34.0 (2026-02-19)
-------------------
* Update changelogs
* Update unstable.yml
* Add tool to validate task composer config
* Only make joint waypoint fixed if not tolerances and the constraint config enabled
* Fix incorrect pipeline input key in config, issue `#705 <https://github.com/tesseract-robotics/tesseract_planning/issues/705>`_
* Added methods to task composer plugin factory to get all available plugin names
* Allow setting minimum delta_t between traj points in ISP solver
* Improve task composer code coverage (`#684 <https://github.com/tesseract-robotics/tesseract_planning/issues/684>`_)
* Fix issue with tolerance size (`#535 <https://github.com/tesseract-robotics/tesseract_planning/issues/535>`_)
* Update to latest trajopt version
* Always return motion planner response (`#710 <https://github.com/tesseract-robotics/tesseract_planning/issues/710>`_)
* Update to latest changes in tesseract and trajopt repo
* Reduce scope of PCL dependency
* Update to leverage trajopt_ifopt dynamic size collision constraint
* Fix remaining ifopt:: namespace usage (`#706 <https://github.com/tesseract-robotics/tesseract_planning/issues/706>`_)
* Fix error specifying substep == num_substeps
* actually set opt params for fix state collision task
* Bump actions/upload-artifact from 5 to 6
  Bumps [actions/upload-artifact](https://github.com/actions/upload-artifact) from 5 to 6.
  - [Release notes](https://github.com/actions/upload-artifact/releases)
  - [Commits](https://github.com/actions/upload-artifact/compare/v5...v6)
  ---
  updated-dependencies:
  - dependency-name: actions/upload-artifact
  dependency-version: '6'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
* Update to leverage trajopt node variable set (`#695 <https://github.com/tesseract-robotics/tesseract_planning/issues/695>`_)
* Add missing opt_params serialization
* Update colcon action to v14
* Bump johnwason/vcpkg-action from 6 to 7
  Bumps [johnwason/vcpkg-action](https://github.com/johnwason/vcpkg-action) from 6 to 7.
  - [Release notes](https://github.com/johnwason/vcpkg-action/releases)
  - [Commits](https://github.com/johnwason/vcpkg-action/compare/v6...v7)
  ---
  updated-dependencies:
  - dependency-name: johnwason/vcpkg-action
  dependency-version: '7'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
* Bump actions/setup-python from 4 to 6
  Bumps [actions/setup-python](https://github.com/actions/setup-python) from 4 to 6.
  - [Release notes](https://github.com/actions/setup-python/releases)
  - [Commits](https://github.com/actions/setup-python/compare/v4...v6)
  ---
  updated-dependencies:
  - dependency-name: actions/setup-python
  dependency-version: '6'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
* Updated Dockerfile to use tmpfs mount (`#649 <https://github.com/tesseract-robotics/tesseract_planning/issues/649>`_)
* Restore optimization callback functionality (`#680 <https://github.com/tesseract-robotics/tesseract_planning/issues/680>`_)
* Bump actions/upload-artifact from 4 to 5
  Bumps [actions/upload-artifact](https://github.com/actions/upload-artifact) from 4 to 5.
  - [Release notes](https://github.com/actions/upload-artifact/releases)
  - [Commits](https://github.com/actions/upload-artifact/compare/v4...v5)
  ---
  updated-dependencies:
  - dependency-name: actions/upload-artifact
  dependency-version: '5'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
* Bump peaceiris/actions-gh-pages from 3 to 4
  Bumps [peaceiris/actions-gh-pages](https://github.com/peaceiris/actions-gh-pages) from 3 to 4.
  - [Release notes](https://github.com/peaceiris/actions-gh-pages/releases)
  - [Changelog](https://github.com/peaceiris/actions-gh-pages/blob/main/CHANGELOG.md)
  - [Commits](https://github.com/peaceiris/actions-gh-pages/compare/v3...v4)
  ---
  updated-dependencies:
  - dependency-name: peaceiris/actions-gh-pages
  dependency-version: '4'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
* Bump docker/build-push-action from 5 to 6
  Bumps [docker/build-push-action](https://github.com/docker/build-push-action) from 5 to 6.
  - [Release notes](https://github.com/docker/build-push-action/releases)
  - [Commits](https://github.com/docker/build-push-action/compare/v5...v6)
  ---
  updated-dependencies:
  - dependency-name: docker/build-push-action
  dependency-version: '6'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
* Bump actions/checkout from 1 to 6
  Bumps [actions/checkout](https://github.com/actions/checkout) from 1 to 6.
  - [Release notes](https://github.com/actions/checkout/releases)
  - [Changelog](https://github.com/actions/checkout/blob/main/CHANGELOG.md)
  - [Commits](https://github.com/actions/checkout/compare/v1...v6)
  ---
  updated-dependencies:
  - dependency-name: actions/checkout
  dependency-version: '6'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
* Cleanup doxygen file headers
* Add dependabot
* Remove old console bridge cmake target logic
* Fix run cpack script
* Remove getStaticKey from profiles
* Exclude test suite code from coverage
* Add time parameterization core tests
* Exclude motion planner test directory from coverage
* Improve time parameterization coverage
* Exclude test directory from coverage
* Improve serialization coverage
* Fix poly type constness (`#682 <https://github.com/tesseract-robotics/tesseract_planning/issues/682>`_)
* Switch to Cereal for serialization (`#681 <https://github.com/tesseract-robotics/tesseract_planning/issues/681>`_)
* Remove boost serialization from task composer plugins
* Contributors: Joseph Schornak, Levi Armstrong, Michael Ripperger, Roelof Oomen, Tyler Marr, dependabot[bot]

0.33.1 (2025-10-29)
-------------------
* Update changelogs
* Update raster tasks to use correct parent data storage
* Add expected key override
* Contributors: Levi Armstrong

0.33.0 (2025-10-28)
-------------------
* Update changelogs
* Update now that osqp_eigen supports osqp v1.0.0 params
* Remove renaming and indexing (`#674 <https://github.com/tesseract-robotics/tesseract_planning/issues/674>`_)
* Add solver settings to fix state collision task (`#672 <https://github.com/tesseract-robotics/tesseract_planning/issues/672>`_)
* Update based on changes in boost plugin loaders latest release
* Fix getRobotConfig utility to work with positioners
* Add missing ONE_PER_STEP check in contactCheckProgram (`#671 <https://github.com/tesseract-robotics/tesseract_planning/issues/671>`_)
* Add and leverage nested data storage to avoid key reuse issues (`#673 <https://github.com/tesseract-robotics/tesseract_planning/issues/673>`_)
  * Add and leverage nested data storage to avoid key reuse issues
  * Remove task name from keys in raster tasks
  * Abort when exception is thrown
* Add For Each Task (`#670 <https://github.com/tesseract-robotics/tesseract_planning/issues/670>`_)
* Contributors: Levi Armstrong, Tyler Marr

0.32.0 (2025-09-10)
-------------------
* Update changelogs
* Set ompl log level based on request verbose flag
* Add custom benchmarking executable for tesseract examples
* Update depends versions in rosinstall files
* Fixed incorrect number of substeps when adding contact (`#668 <https://github.com/tesseract-robotics/tesseract_planning/issues/668>`_)
* Print a collision summary to status of failed contact check (`#543 <https://github.com/tesseract-robotics/tesseract_planning/issues/543>`_)
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Add trajopt collision coefficients to fix state collision profile (`#655 <https://github.com/tesseract-robotics/tesseract_planning/issues/655>`_)
* Remove focal from CI (removed from Trajopt)
* Remove unsupported windows-2019 from CI
* Update OSQP dependency to v1.0.0 and OSQPEigen to v0.10.3
* Clean up ompl yaml tests
* Add trajopt profile yaml extensions (`#661 <https://github.com/tesseract-robotics/tesseract_planning/issues/661>`_)
* Add utility to make time continuously increasing
* Fix invalid sampling in sampleToolAxis
* Improve memory allocations
* Update examples to only print program when debug is enabled
* Added Profiles constructors for YAML conversions (`#652 <https://github.com/tesseract-robotics/tesseract_planning/issues/652>`_)
* Upgrade descartes to version 0.4.8
* Add SimplePlannerLVSAssignPlanProfile, SimplePlannerLVSAssignNoIKPlanProfile, SimplePlannerFixedSizeAssignPlanProfile (with IK), rename SimplePlannerFixedSizeAssignNoIKPlanProfile (`#448 <https://github.com/tesseract-robotics/tesseract_planning/issues/448>`_)
* Update message in SetDigitalInstruction
* Add trajopt joint configs for fix state collision task (`#637 <https://github.com/tesseract-robotics/tesseract_planning/issues/637>`_)
* Fix error and success reporting in examples
* Add Set Digital Instruction
* Fix missing export in kinematic_limits_check_task.h
* Contributors: John Wason, Levi Armstrong, Roelof Oomen, Samantha Smith, Tyler Marr

0.31.0 (2025-07-06)
-------------------
* Update changelogs
* Fix yaml extensions file name spelling
* Add OMPL collision cost objective
* Improve processYamlIncludeDirective peformance
* Removed redundant edge evaluator check for cartesian waypoint (`#628 <https://github.com/tesseract-robotics/tesseract_planning/issues/628>`_)
* Clang formatting
* Correctly update input key rather than output key for raster task
* Add check to for missing velocity and acceleration data in kinematic limits check task
* Fix constant tcp speed parameterization not populating first state
* Fix descartes profile serialization (`#634 <https://github.com/tesseract-robotics/tesseract_planning/issues/634>`_)
* Add missing contact_manager_config in FixStateCollisionProfile setialization
* Run apt update before installing packages (`#630 <https://github.com/tesseract-robotics/tesseract_planning/issues/630>`_)
* TOTG profile: Restore constructor with parameters
* Update constant tcp speed parameterization duration calcuation method
* Add constant tcp speed parameterization task
* Add constant tcp speed parameterization leveraging kdl library
* Update time parameterization interface to leverage profiles to support cartesian time parameterization
* Port KinematicLimitsCheckTask from SNP Workshop
* Implement the option to format result as output for SimplePlanner (`#622 <https://github.com/tesseract-robotics/tesseract_planning/issues/622>`_)
  * Implement the option to format result as output for SimplePlanner
  * Fix format_result_as_input default for generateInterpolatedProgram
  * Fix format_result_as_input default for MinLengthTask
* Fix createCartesianPositionConstraint not using coeffs
* Remove PluginLoader and ClassLoader from tesseract_common fwd.h
* Make sure serialized objects have friend struct tesseract_common::Serialization
* Contributors: Levi Armstrong, Michael Ripperger, Roelof Oomen, Tyler Marr

0.30.0 (2025-04-23)
-------------------
* Update changelogs
* Add boost plugin loader to windows rosinstall
* Move profile dictionary to tesseract_common
* Rename PlanProfile to MoveProfile
* Allow enabling OSQP workspace update via settings
* Moved opt_params to the TrajOptIfoptSolverProfile class to match the structure of TrajOptSolverProfile/TrajOpOSQPSolverProfile (`#608 <https://github.com/tesseract-robotics/tesseract_planning/issues/608>`_)
* Remove use of use_weighted_sum in trajopt config
* Update cmake format CI to 22.04 image
* Update to leverage boost_plugin_loader
* Store OSQPEigenSettings as unique_ptr (`#610 <https://github.com/tesseract-robotics/tesseract_planning/issues/610>`_)
* Demangle profile class name
* Added class type to task composer dotgraph description
* Removed unnecessary asserts in time parameterization utility
* Swap descartes assert with motion planning failure (`#606 <https://github.com/tesseract-robotics/tesseract_planning/issues/606>`_)
* Fix codecov CI
* Fix issues with how Collision Check Config was being used
* Update to changes with legacy trajopt leveraging trajopt_common::TrajOptCollisionConfig
* Add misc-include-cleaner.IgnoreHeaders
* Contributors: Levi Armstrong, Michael Ripperger, Roelof Oomen

0.29.1 (2025-03-21)
-------------------
* Update changelogs
* Store OSQPEigenSettings by value
* Contributors: Levi Armstrong

0.29.0 (2025-03-20)
-------------------
* Update changelogs
* Fix: deleted default constructors do not allow serialization (`#600 <https://github.com/tesseract-robotics/tesseract_planning/issues/600>`_)
* Simplify the use of poly types
* Add constructors to WaypointPoly to support State, Joint and Cartesian Interface class
* Update because of changes with AnyPoly
* Fix clang-tidy errors
* Add documentation to instruction poly types
* Leverage inheritance over type erasure for instructions
* Add documentation to waypoint poly types
* Remove no longer needed methods from the move instruction
* Leverage inheritance over type erasure for waypoints
* Remove duplicated serialize from different class (`#595 <https://github.com/tesseract-robotics/tesseract_planning/issues/595>`_)
* Leverage tesseract_collision CollisionEvaluatorType
* Update to leverage std::filesystem
* Update to clang-tidy-17 (`#591 <https://github.com/tesseract-robotics/tesseract_planning/issues/591>`_)
* Update colcon action to v10
* Remove unimplemented method TaskComposerContext abort that takes a node
* Add python specific method to task composer graph and server
* Fix isValidState in planner_utils.h
* Remove use of TaskComposerNodeInfo::UPtr and just use TaskComposerNodeInfo instead
* Fix missing include for use of shared_ptr
* Contributors: Joseph Schornak, Levi Armstrong, Roelof Oomen

0.28.4 (2025-01-22)
-------------------
* Update changelogs
* Add ruckig to cpack script
* Update task composer examples to leverage resource url
* Contributors: Levi Armstrong

0.28.3 (2025-01-21 18:26)
-------------------------
* Update changelogs
* Fix pick and place example
* Contributors: Levi Armstrong

0.28.2 (2025-01-21 14:17)
-------------------------
* Update changelogs
* Fix cpack and add ruckig cpack generation and upload (`#585 <https://github.com/tesseract-robotics/tesseract_planning/issues/585>`_)
* Contributors: Levi Armstrong

0.28.1 (2025-01-16 15:04)
-------------------------
* Update changelogs
* Fix package debian CI pipeline
* Contributors: Levi Armstrong

0.28.0 (2025-01-16 14:11)
-------------------------
* Update changelogs
* Add cpack debian package CI pipeline (`#582 <https://github.com/tesseract-robotics/tesseract_planning/issues/582>`_)
* Fix cpack build and enable during CI pipeline
* Update depends
* Remove manip_info as option task input
* Fix motion planner task to support error branching
* Fix cpack name when components contain underscore
* Include task composer in .run-cpack, fix typos in CMakeLists (`#577 <https://github.com/tesseract-robotics/tesseract_planning/issues/577>`_)
* Fix MSVC compiler error in TaskComposerPluginFactory
* Leverage tesseract_common loadYamlFile and loadYamlString
* Modify composite instruction constructor
* Update tesseract collision validator
* Fix ompl so it only simplifies when requested (`#562 <https://github.com/tesseract-robotics/tesseract_planning/issues/562>`_)
* Fix composite and move instruction equal operator and serialization (`#561 <https://github.com/tesseract-robotics/tesseract_planning/issues/561>`_)
* Changes required now that environment return const shared pointers for getKinematicGroup and getJointGroup
* Fix ompl use of simplify
* New TrajOpt IFOPT profiles
* Improve descartes plan profile to include sample min, max and ik solver setting
* Add new TrajOpt Profiles
* Fix formatJointPosition handle of cartesian seed
* Add simple ompl profile interface
* Fixed clang-format in command language test suite
* Update descartes profiles (`#546 <https://github.com/tesseract-robotics/tesseract_planning/issues/546>`_)
* Do not delete special member functions (`#551 <https://github.com/tesseract-robotics/tesseract_planning/issues/551>`_)
* Add Ruckig composite profile serialization
* Added issue templates (`#549 <https://github.com/tesseract-robotics/tesseract_planning/issues/549>`_)
* Remove PlanningRequest form simple profile interface
* Remove env_state from planning request
* Fix windows build (`#542 <https://github.com/tesseract-robotics/tesseract_planning/issues/542>`_)
  Co-authored-by: John Wason <wason@wasontech.com>
* Address some windows build issues (`#541 <https://github.com/tesseract-robotics/tesseract_planning/issues/541>`_)
* Modify the concept of profile overrides in Composite and Move Instruction
* Add boost serialization to profiles
* Add profile base class and update profile dictionary to not leverage template methods
* Update dependencies.repos (`#536 <https://github.com/tesseract-robotics/tesseract_planning/issues/536>`_)
* Contributors: John Wason, Levi Armstrong, Max DeSantis, Michael Ripperger, Roelof Oomen

0.27.0 (2024-12-01)
-------------------
* Update changelogs
* Simplify the type erasure poly
* Add FormatPlanningInputTask
* Update online example to pause every 10 seconds
* Update to leverage clang-format-12
* Update online planning example and toJointTrajectory to leverage trajectory uuid
* Contributors: Levi Armstrong

0.26.1 (2024-10-29)
-------------------
* Update changelogs
* Fix format as input task to handle both joint and state waypoints
* Contributors: Levi Armstrong

0.26.0 (2024-10-27)
-------------------
* Update changelogs
* Fix use of trajopt composite profile cost and constraint collision config
* Improve toToolPath to handle nested composites
* Fix KinematicGroupInstructionInfo and JointGroupInstructionInfo extract cartesian pose
* Store dotgraph in TaskComposerLog
* Remove TesseractSupportResourceLocator
* Fix Ubuntu CI by extracting latest Tag instead of organization variable
* Add additional constructor to simple planner interpolation tools
* Update ubuntu.yml
* Contributors: Levi Armstrong

0.25.1 (2024-09-29 09:10)
-------------------------
* Update changelogs
* Update ubuntu.yml
* Contributors: Levi Armstrong

0.25.0 (2024-09-29 08:27)
-------------------------
* Update changelogs
* Update docker CI so it can be triggered
* Rename Timer to Stopwatch
* Contributors: Levi Armstrong

0.24.2 (2024-08-19)
-------------------
* Update changelogs
* Add optional output to task which produce contact results
* Contributors: Levi Armstrong

0.24.1 (2024-08-16)
-------------------
* Update changelogs
* Add getRootNode to TaskComposerGraph and update TaskComposerNodeInfo to store root_uuid
* Contributors: Levi Armstrong

0.24.0 (2024-08-14)
-------------------
* Update changelogs
* Update TaskComposerNodeInfo to allow searching graph
* Fix RemapTask support for global remapping
* Add data storage to task composer node info
* Contributors: Levi Armstrong

0.23.6 (2024-08-06)
-------------------
* Update changelogs
* Make aborted nodes grey in dotgraph
* Fix task composer context serialization
* Fix Mac OS clang compiler warnings (`#496 <https://github.com/tesseract-robotics/tesseract_planning/issues/496>`_)
* Contributors: John Wason, Levi Armstrong

0.23.5 (2024-08-01)
-------------------
* Update changelogs
* Update so pipelines can have child graph tasks
* Fix Windows and Mac OS Github Actions (`#489 <https://github.com/tesseract-robotics/tesseract_planning/issues/489>`_)
* Contributors: John Wason, Levi Armstrong

0.23.4 (2024-07-29)
-------------------
* Update changelogs
* Fix task composer graph config loading
* Contributors: Levi Armstrong

0.23.3 (2024-07-28)
-------------------
* Update changelogs
* Cleanup boost serialization (`#490 <https://github.com/tesseract-robotics/tesseract_planning/issues/490>`_)
* Contributors: Levi Armstrong

0.23.2 (2024-07-25)
-------------------
* Update changelogs
* Make dotgraph label nojustify
* Fix task composer data storage constructors
* Contributors: Levi Armstrong

0.23.1 (2024-07-24 21:13)
-------------------------
* Update changelogs
* Update ubuntu.yml
* Contributors: Levi Armstrong

0.23.0 (2024-07-24 20:07)
-------------------------
* Update changelogs
* Add name to data storage to hold pipeline name
* Improve dotgraph
* Add boost serialization support to Profile dictionary
* Fix discrete and continuous task info constructors
* Remove task_composer_problem.h
* Add check for expected keys to TaskComposerGraph
* Update task composer readme
* Do not export plugin libraries (`#474 <https://github.com/tesseract-robotics/tesseract_planning/issues/474>`_)
* Add CI for Ubuntu Noble (`#467 <https://github.com/tesseract-robotics/tesseract_planning/issues/467>`_)
* Update RICB to version 0.6.2
* Make disabled task dotgraph color yellow
* Environment should be stored as const in data storage
* fix applyCorrectionWorkflow definition
* Remove TaskComposerProblem and leverage TaskComposerDataStorage instead (`#469 <https://github.com/tesseract-robotics/tesseract_planning/issues/469>`_)
* Fixes for building on Ubuntu Noble
* Contributors: Levi Armstrong, Roelof Oomen

0.22.1 (2024-06-12)
-------------------
* Update changelog
* Fix: Add cost to cost_infos instead of cnt_infos
* Contributors: Levi Armstrong, Roelof Oomen

0.22.0 (2024-06-10)
-------------------
* Update changelogs
* Improve FixStateCollisionTask by also add a cost (distance) between new and original joint state
* Add task composer graph validation (`#463 <https://github.com/tesseract-robotics/tesseract_planning/issues/463>`_)
* More compact descartes collision logging output (`#460 <https://github.com/tesseract-robotics/tesseract_planning/issues/460>`_)
* Add HasDataStorageEntryTask and FormatAsResultTask with unit tests (`#462 <https://github.com/tesseract-robotics/tesseract_planning/issues/462>`_)
* Upgrade ros_industrial_cmake_boilerplate to 0.6.0
* Better debugging feedback on failed Descartes plan (`#401 <https://github.com/tesseract-robotics/tesseract_planning/issues/401>`_)
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Update CI docker tag to 0.22.X (`#459 <https://github.com/tesseract-robotics/tesseract_planning/issues/459>`_)
* Add convex_solver_settings to TrajOptIfoptDefaultSolverProfile (`#425 <https://github.com/tesseract-robotics/tesseract_planning/issues/425>`_)
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Add include for Ubuntu Noble
* Fix descartes planner check if the graph built
* Add status_code to TaskComposerNodeInfo
* Add time parameterization interface (`#455 <https://github.com/tesseract-robotics/tesseract_planning/issues/455>`_)
* Add utility methods to task composer node info container
* Update to use forward declarations (`#449 <https://github.com/tesseract-robotics/tesseract_planning/issues/449>`_)
* Update to vcpkg-action@v6 and use vcpkg-binarycache (`#434 <https://github.com/tesseract-robotics/tesseract_planning/issues/434>`_)
* Add toleranced waypoints to TrajOpt Solver (`#403 <https://github.com/tesseract-robotics/tesseract_planning/issues/403>`_)
* Use trajopt docker
* Update trajopt ifopt to support dynamic cartesian waypoints
* Fixed issue checking wrong size for contact check results
* Base code quality CI on 'unstable' (master) tesseract docker container (`#442 <https://github.com/tesseract-robotics/tesseract_planning/issues/442>`_)
* Update matching https://github.com/tesseract-robotics/tesseract/pull/989
* Feat/more verbose planning failures (`#440 <https://github.com/tesseract-robotics/tesseract_planning/issues/440>`_)
* Adding Trajopt_Ifopt option to all examples (`#389 <https://github.com/tesseract-robotics/tesseract_planning/issues/389>`_)
* Fix error message for missing output keys
* Contributors: John Wason, Levi Armstrong, Roelof, Roelof Oomen, Tyler Marr

0.21.7 (2024-02-03)
-------------------
* Update changelog
* Fix simple planner enforcing limits on cartesian waypoints that do not have a seed
* Add optional namespace field to task nodes (`#433 <https://github.com/tesseract-robotics/tesseract_planning/issues/433>`_)
* Remove deprecated AnalyzeTemporaryDtors
  See https://github.com/llvm/llvm-project/issues/62020
* Contributors: Levi Armstrong, Roelof, Tyler Marr

0.21.6 (2023-12-21)
-------------------
* Update changelog
* Fix TOTG state time from start numerical issue (`#431 <https://github.com/tesseract-robotics/tesseract_planning/issues/431>`_)
* Remove include (fixes todo)
* Add Mac OSX support (`#428 <https://github.com/tesseract-robotics/tesseract_planning/issues/428>`_)
* Contributors: John Wason, Levi Armstrong, Roelof

0.21.5 (2023-12-13)
-------------------
* Update changelog
* Fix TaskComposerProblem serialization and equal operator
* Contributors: Levi Armstrong

0.21.4 (2023-11-21)
-------------------
* Update changelogs
* Add clear method to profile dictionary
* Contributors: Levi Armstrong

0.21.3 (2023-11-20)
-------------------
* Update changelogs
* Update readme
* Update colcon action version
* Add Docker CI
* Remove use of Industrial CI (`#420 <https://github.com/tesseract-robotics/tesseract_planning/issues/420>`_)
* Added docker files (`#414 <https://github.com/tesseract-robotics/tesseract_planning/issues/414>`_)
* Update README.rst
  Description of Simple Motion Planner task fixed
* Contributors: Levi Armstrong, Michael Ripperger, Roelof

0.21.2 (2023-11-17 14:47)
-------------------------
* Update changelogs
* Improve dynamic tasking support
* Contributors: Levi Armstrong

0.21.1 (2023-11-17 11:34)
-------------------------
* Update changelog
* Fix loss of first waypoint in upsample trajectory (`#416 <https://github.com/tesseract-robotics/tesseract_planning/issues/416>`_)
* Use taskflow subflow for graph execution to allow timing of execution
* Contributors: Levi Armstrong, Thomas Hettasch

0.21.0 (2023-11-10)
-------------------
* Update changelogs
* Update dependencies.repos (`#411 <https://github.com/tesseract-robotics/tesseract_planning/issues/411>`_)
* Revert "Changed CI base directory to /opt" (`#413 <https://github.com/tesseract-robotics/tesseract_planning/issues/413>`_)
  This reverts commit 9377b1d5d9d4dfe9d07dc45e3f532b0295911a1d.
* Changed CI base directory to /opt
* Fix clang-tidy errors
* Pin windows vcpkg version
* Replace input_indexing and output_indexing with indexing
* Replace input_remapping and output_remapping with remapping
* Updated to version of ICI with VCS shallow cloning (`#405 <https://github.com/tesseract-robotics/tesseract_planning/issues/405>`_)
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Move TaskComposerProblem input to base class and change type to tesseract_common::AnyPoly
* remove results from TaskComposerNodeInfo
* Fix contact check program to support joint and state waypoints
* Unused includes cleanup
* Update based on changes in trajopt
* Contributors: Levi Armstrong, Michael Ripperger, Roelof Oomen

0.20.1 (2023-10-02)
-------------------
* Update changelogs
* Fix CI ubuntu.yml
* Update composite instruction user data to align with gazebo user data variant
* Contributors: Levi Armstrong

0.20.0 (2023-09-29)
-------------------
* Update changelogs
* Update CI to use Tesseract version 0.20.0
* Update tesseract examples
* Remove AbortTask
* Add input instruction to planning problem
* Add user data support to CompositeInstruction
* Added jammy build
* Merge pull request `#370 <https://github.com/tesseract-robotics/tesseract_planning/issues/370>`_ from marip8/update/task-composer-factory-constructor
  Add new task composer plugin factory constructor
* Added unit test for new TaskComposerPluginFactory constructor
* Update add_sources.sh
* Added constructor to task composer plugin factory to use task composer plugin config struct
* Install command language test suite
* Rename TaskComposerInput to TaskComposerContext and simplify interfaces (`#379 <https://github.com/tesseract-robotics/tesseract_planning/issues/379>`_)
* Contributors: Levi Armstrong, Michael Ripperger

0.19.1 (2023-09-16)
-------------------
* Minor improvements and debug info in task composer (`#378 <https://github.com/tesseract-robotics/tesseract_planning/issues/378>`_)
* Add seed IK for base instruction to SimpleMotionPlanner (fixes todo)
* Fixed missing IK solver initialization of KinematicGroupInstructionInfo
* Contributors: Levi Armstrong, Roelof Oomen

0.19.0 (2023-09-05)
-------------------
* Update changelogs
* Update to leverage cmake components
* Contributors: Levi Armstrong

0.18.5 (2023-09-01)
-------------------
* Initial support for a TrajOptIfoptSolverProfile (`#354 <https://github.com/tesseract-robotics/tesseract_planning/issues/354>`_)
* Moved simple planner into own sub-directory
* Disable GlassUprightTrajOptIfoptExampleUnit Test
* Fix Raster and RasterOnly Tasks
* Fix TrajOpt Ifopt collision cost and constraint naming
* Fixed issue where tesseract_common_trajectory.cpp wasn't getting built (`#356 <https://github.com/tesseract-robotics/tesseract_planning/issues/356>`_)
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Add elapsed time for pipelines and include in dot graph
* Add conditional to subgraph in dot graph output
* Add input and output keys to dot graph
* Add Remap Task (`#351 <https://github.com/tesseract-robotics/tesseract_planning/issues/351>`_)
* Merge pull request `#300 <https://github.com/tesseract-robotics/tesseract_planning/issues/300>`_ from marip8/update/docs-build
  Fixed documentation build job
* Merge pull request `#350 <https://github.com/tesseract-robotics/tesseract_planning/issues/350>`_ from marip8/update/ci
  Reduce CI-built docker image size
* Removed upstream and target workspace directories after CI build
* Use tesseract organization fork of ICI
* Updated location of logo and doxygen config
* Contributors: Levi Armstrong, Michael Ripperger, Roelof, Tyler Marr

0.18.4 (2023-07-07)
-------------------
* Update changelogs
* Move task composer elapse timing to base classes
* Contributors: Levi Armstrong

0.18.3 (2023-07-04)
-------------------
* Update changelog
* Fix MotionPlannerTaskInfo serialization
* Contributors: Levi Armstrong

0.18.2 (2023-07-03 17:09)
-------------------------
* Update changelogs
* Add InstructionPoly setUUID method with tests
* Add clone method to TaskComposerProblem
* Contributors: Levi Armstrong

0.18.1 (2023-07-03 10:59)
-------------------------
* Update changelogs
* Fix TaskComposerServer destruction
* Contributors: Levi Armstrong

0.18.0 (2023-06-30)
-------------------
* Update changelogs
* Update task_composer_plugins_no_trajopt_ifopt.yaml
* Restruct Raster yaml config to have same look as everything else
* Leverage AbortTask and make ErrorTask not abort
* Remove unused file
* Upgrade to TrajOpt 0.6.0
* Fixed actual term number check being 1 more than stated number (`#333 <https://github.com/tesseract-robotics/tesseract_planning/issues/333>`_)
* Add task composer planning unit tests (`#341 <https://github.com/tesseract-robotics/tesseract_planning/issues/341>`_)
* Fixes for Python wrappers (`#329 <https://github.com/tesseract-robotics/tesseract_planning/issues/329>`_)
* Add TaskComposerServer unit tests
* Add task composer taskflow unit tests (`#339 <https://github.com/tesseract-robotics/tesseract_planning/issues/339>`_)
* Add TaskComposerPipeline and improve task composer code coverage (`#337 <https://github.com/tesseract-robotics/tesseract_planning/issues/337>`_)
* Added trajectory logger printout to trajectory checker (`#338 <https://github.com/tesseract-robotics/tesseract_planning/issues/338>`_)
* Add TrajOpt multi threaded support
* Added an extra needed #include for 22.04 builds (`#332 <https://github.com/tesseract-robotics/tesseract_planning/issues/332>`_)
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Add test suite to command language and improve code coverage (`#331 <https://github.com/tesseract-robotics/tesseract_planning/issues/331>`_)
* Restructure tesseract_task_composer like other plugin based packages
* Add PlanningTaskComposerProblem
* Added ability to colorize dotgraphs with planning results (`#327 <https://github.com/tesseract-robotics/tesseract_planning/issues/327>`_)
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Contributors: John Wason, Levi Armstrong, Tyler Marr

0.17.0 (2023-06-06)
-------------------
* Update changelogs
* Fix serialization typo of CollisionCostConfig.safetymargin to BufferMargin (`#323 <https://github.com/tesseract-robotics/tesseract_planning/issues/323>`_)
* Fix Key Naming Scheme in Raster Motion Task  (`#324 <https://github.com/tesseract-robotics/tesseract_planning/issues/324>`_)
  @marrts Great find and thanks for the fix.
* Update depends
* Fix incorrect include path in tesseract_common_trajectory.h
* Fix composite instruction iterator construction to call default constructor
* Fix task composer cmake plugins variable
* Update task nodes to on failure store input in output location to better support error branching
* Fix some typos
* Contributors: Levi Armstrong, Roelof, Roelof Oomen, Tyler Marr

0.16.3 (2023-05-03)
-------------------
* Update changelogs
* Fix FormatAsInputTask to store results
* Contributors: Levi Armstrong

0.16.2 (2023-04-28)
-------------------
* Update changelogs
* Update dependencies_windows_build.repos
* Add name to waypoints
* Contributors: Levi Armstrong

0.16.1 (2023-04-11)
-------------------
* Update changelogs
* Update depends
* Contributors: Levi Armstrong

0.16.0 (2023-04-09)
-------------------
* Update changelogs
* Update depends in CI and rosinstall
* Update readme
* Fix codecov CI after script
* Update contactCheckProgram and copy unit tests from tesseract_environment
* Add FormatAsInputTask
* Update to leverage CollisionCheckProgramType in collision config
* Fix clang-tidy errors
* Update to support new contact results class (`#297 <https://github.com/tesseract-robotics/tesseract_planning/issues/297>`_)
* This fixes crash in ompl::geometric::SimpleSetup::simplifySolution() (`#298 <https://github.com/tesseract-robotics/tesseract_planning/issues/298>`_)
* Fixing OMPL freespace example and a typo (`#299 <https://github.com/tesseract-robotics/tesseract_planning/issues/299>`_)
  * Fix freespace OMPL example (was hybrid)
  * Fix typo in iterative_spline_parameterization_profile file name
* CI Update (`#288 <https://github.com/tesseract-robotics/tesseract_planning/issues/288>`_)
* Add TOTG Node Info class
* Contributors: Levi Armstrong, Michael Ripperger, Roelof, afrixs

0.15.5 (2023-03-22)
-------------------
* Update changelogs
* Update depends in rosinstall
* Fix numerical issue in totg
* Update toJointTrajectory to include cartesian waypoint if seed exists
* Fix TOTG assignData
* Add fix_state_collision clone method and serialize contact results
* Build fixes for Focal/Foxy and Jammy/Humble
* Contributors: Levi Armstrong, Roelof Oomen

0.15.4 (2023-03-16)
-------------------
* Update changelogs
* Update example to include geometry headers
* Contributors: Levi Armstrong

0.15.3 (2023-03-15)
-------------------
* Update changelogs
* Update trajopt version
* Add support for cartesian waypoint type in getJointPosition and getJointNames
* Contributors: Levi Armstrong

0.15.2 (2023-03-14)
-------------------
* Update changelogs
* Update dependencies.rosinstall
* Update dependencies_windows_build.rosinstall
* Clean up task composer serialization
* Contributors: Levi Armstrong

0.15.1 (2023-03-09)
-------------------
* Update changelog
* Add assert checks to simple planner
* Add method for retrieving task from TaskComposerServer
* Use try catch in TaskComposerTask run because exceptions are not propagated in multi threaded runs.
* Update fix state bounds task to ignore cartesian waypoint types
* Make MoveInstruction constructors explict and add one for WaypointPoly
* Remove old unused simple_planner_utils.cpp
* Contributors: Levi Armstrong

0.15.0 (2023-03-03)
-------------------
* Update changelog
* Update rosinstall depends
* Update task composer to leverage plugins (`#282 <https://github.com/tesseract-robotics/tesseract_planning/issues/282>`_)
* Use templates for raster task to reduce code duplications (`#279 <https://github.com/tesseract-robotics/tesseract_planning/issues/279>`_)
* Fix TrajOpt IFOPT code coverage build
* Add descartes no post check motion pipeline task
* clean up update end state task
* remove commented code from ompl motion planner
* Fix descartes default plan profile
* Fix descartes global motion pipeline task
* Merge pull request `#269 <https://github.com/tesseract-robotics/tesseract_planning/issues/269>`_ from marip8/update/time-param-org
  Added optional builds of time parameterization implementations
* Created separate targets for each time parameterization implementation
* Updated task composer package
* Moved core into own subdirectory; moved headers into specific sub-directories; updated unit tests
* Update tesseract_time_parameterization-config.cmake.in
* Added optional builds of time parameterization implementations
* Fix descartes motion planner handling of unconstrained joint waypoints
* Remove composite start instruction
* Fix descartes collision edge evaluator
* Add toJointTrajectory overload for InstructionPoly
* Add uuid and parent_uuid to InstructionPoly (`#261 <https://github.com/tesseract-robotics/tesseract_planning/issues/261>`_)
* Update dependency tags (`#258 <https://github.com/tesseract-robotics/tesseract_planning/issues/258>`_)
* Contributors: Levi Armstrong, Michael Ripperger

0.14.0 (2022-10-23)
-------------------
* Fix tesseract_task_composer version
* Update changelog
* Add ompl to default tasks utility function
* Fix trajopt ifopt task name
* Update to use ModifyAllowedCollisionCommand
* Add environment to TaskComposerNodeInfo
* Add method to TaskComposerDataStorage to get copy of all data
* Update TaskComposerNodeInfo contructor to take node type
* Fix trajopt ifopt planner
* Remove tesseract_process_managers package
* Add convex solver config to TrajOpt solver profile and fix puzzle piece aux example
* Remove references to tesseract process managers package
* Fixes for Ubuntu 22.04 (boost and mutex)
* Add tesseract_task_composer package to replace tesseract_process_managers
* Fix clang tidy errors
* Rename TransitionMuxTask to UpdateStartAndEndStateTask
* Add TaskComposerServer
* Add task composer problem
* Remove clone method from TaskComposerNode
* Finish migrating unit tests
* Break up task to avoid configuration parameters
* Update task to require returning TaskComposerNodeInfo
* Fix raster global tasks
* Cleanup task composer examples
* Add remaining raster tasks
* Fix rebase conflicts
* Fix clang-tidy errors
* Store input and output keys in TaskComposerNode
* Add clone method to TaskComposerNode
* Add TaskComposerPluginFactory
* Cleanup TaskComposerFuture
* Move contents of taskflow_utils.h into taskflow executor
* Add reset capability to TaskComposerInput
* Remove executor from TaskComposerInput
* Add TaskComposerExecutor and TaskComposerFuture
* Add inbound edges to TaskComposerNode
* Fix dot graph generation
* Rename SeedMinLengthTask to MinLengthTask
* Fix task composer seed_min_length_task
* Update interpolation to return vector of MoveInstructionPoly
* Move the interpolate functions into its own file and add StartTask need for raster task
* Add dump function to create dot graph
* Add raster motion task
* Update TaskComposerGraph to use task uuid as key for nodes
* Add TaskComposerTask class
* Add motion planning pipelines to tesseract_task_composer
* Add format_result_as_input to PlannerRequest
* Fix cmake files
* Add conditional task type
* Add done and error task
* Rename TaskComposerPipeline to TaskComposerGraph
* Update tesseract process managers unit to use legacy version
* Move name and check input into motion planners base class
* Rename generateSeed to generateInterpolatedProgram
* Update tesseract_process_managers to use legacy planners
* Fix descartes and simple planner unit tests
* Add new trajopt ifopt planner and rename existing one legacy
* Add new trajopt planner and rename existing one legacy
* Add new ompl planner and rename existing one legacy
* Add new descartes planner and rename existing one legacy
* Add new simple planner and rename existing one legacy
* Add isConstrained to JointWaypoint
* Add transition mux task
* Add equal operators to task composer tasks
* Remove use of tesseract_common::StatusCode
* Add task composer package
* Including <boost/serialization/library_version_type.hpp> for Boost 1.74. Fixes `tesseract-robotics/tesseract#764 <https://github.com/tesseract-robotics/tesseract/issues/764>`_
* Contributors: Levi Armstrong, Roelof Oomen

0.13.1 (2022-08-30)
-------------------
* Update changelog
* Add back profile overrides to MoveInstruction
* Contributors: Levi Armstrong

0.13.0 (2022-08-25)
-------------------
* Update changelog
* Fixed bug that wouldn't pass through a 'found' flag
* Add Eigen alignment macro to classes that contain SceneState
* Update clang-tidy, codecove, nightly CI to use master docker
* Add ruckig trajectory smoothing
* Update type erasure benchmarks
* Update simple planners to leverage createChild method
* Add UUID to the MoveInstructionPoly interface
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
* Enable clang-tidy for unstable build
* Remove plan instruction
* Update unit tests
* Update tesseract tag
* Update code based on clang-tidy-14
* Add unit tests for fix_state_bounds_task_generator
* update to leverage limits utility function in tesseract_common
* Contributors: John Wason, Levi Armstrong, Matthew Powelson, Tyler Marr

0.12.0 (2022-07-07)
-------------------
* Update changelog
* Update tesseract tag
* Update ros_industrial_cmake_boilerplate to 0.3.0 (`#214 <https://github.com/tesseract-robotics/tesseract_planning/issues/214>`_)
* Move use of BOOST_CLASS_VERSION to header
* Add ability to submit ProcessPlanningProblem to planning server (`#213 <https://github.com/tesseract-robotics/tesseract_planning/issues/213>`_)
* Set the default descartes behavior to treat all states equally (`#209 <https://github.com/tesseract-robotics/tesseract_planning/issues/209>`_)
* Added CPack (`#208 <https://github.com/tesseract-robotics/tesseract_planning/issues/208>`_)
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Add support for sco::Optimizer::Callbacks to the trajopt solver profile (`#207 <https://github.com/tesseract-robotics/tesseract_planning/issues/207>`_)
* Renames in type erasure to avoid WIN32 defines
* Fix Typos
  - TrajOptMotionPlanner
  - DefaultProcessManagers
* Contributors: John Wason, Levi Armstrong, Michael Ripperger, christian.petersmeier, marrts

0.11.0 (2022-06-20)
-------------------
* Update changelog
* Update to use new type erasure interface (`#203 <https://github.com/tesseract-robotics/tesseract_planning/issues/203>`_)
  * Use new type erasure interface
  * Update waypoint to new type erasure interface and add benchmark
  * Add benchmark comparing type erasure to using unique_ptrs
* Fix issue `#201 <https://github.com/tesseract-robotics/tesseract_planning/issues/201>`_ excess collision checking in contactCheckProgram with ContinuousContactManager
* Contributors: Levi Armstrong

0.10.4 (2022-06-03)
-------------------
* Update changelog
* Update toJointTrajectory to support plan instructions
* Contributors: Levi Armstrong

0.10.3 (2022-05-31)
-------------------
* Update changelog
* Update codecov CI to use noetic
* Update rosinstall files
* Switch test to use tcmalloc_minimal
* Add valid method to ProcessPlanningFuture
* Contributors: Levi Armstrong

0.10.2 (2022-05-24)
-------------------
* Update changelog
* Make process planning server threadsafe and support multiple executors (`#197 <https://github.com/tesseract-robotics/tesseract_planning/issues/197>`_)
* Contributors: Levi Armstrong

0.10.1 (2022-05-09)
-------------------
* Update changelog
* Add name to process planning problem
* Contributors: Levi Armstrong

0.10.0 (2022-05-03)
-------------------
* Update changelogs
* Remove duplicate serialization methods and use those in tesseract_common
* Store environment in process planning problem (`#192 <https://github.com/tesseract-robotics/tesseract_planning/issues/192>`_)
  * Store environment in process planning problem
  * Update readme dependency versions
* Add SWIG %shared_ptr to ProcessPlanningProblem (`#189 <https://github.com/tesseract-robotics/tesseract_planning/issues/189>`_)
* Contributors: John Wason, Levi Armstrong

0.9.9 (2022-04-22)
------------------
* Update changelog
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
* Update changelog
* Update/joint trajectory (`#186 <https://github.com/tesseract-robotics/tesseract_planning/issues/186>`_)
  * Store description when converting from composite to joint trajectory
  * Update windows rosinstall
  * Update rosinstall files
* Contributors: Levi Armstrong

0.9.7 (2022-04-08)
------------------
* Update changelog
* Fix logic in FixStateBounds for case ALL
* Contributors: Levi Armstrong, Matthew Powelson

0.9.6 (2022-04-01)
------------------
* Set tesseract_examples initial version
* Update changelog
* Add tesseract_examples package
* Fix issue in contact_check_profile not setting override type in default constructor
* Contributors: Levi Armstrong

0.9.5 (2022-03-31)
------------------
* Update changelog
* Update to leverage TesseractSupportResourceLocator (`#181 <https://github.com/tesseract-robotics/tesseract_planning/issues/181>`_)
  * Update to leverage TesseractSupportResourceLocator
  * Update CI docker tag to 0.9
* Add ctest calls to windows_noetic_build.yml (`#176 <https://github.com/tesseract-robotics/tesseract_planning/issues/176>`_)
* Add check for time increasing in TOTG (`#175 <https://github.com/tesseract-robotics/tesseract_planning/issues/175>`_)
* Fix windows CI build (`#178 <https://github.com/tesseract-robotics/tesseract_planning/issues/178>`_)
* Contributors: John Wason, Levi Armstrong

0.9.4 (2022-03-25)
------------------
* Update changelog
* Add serialization for ProcessPlanningRequest (`#174 <https://github.com/tesseract-robotics/tesseract_planning/issues/174>`_)
  * Switch serialization instantiations to use tesseract_common macro
  * Add serialization for ProcessPlanningRequest
  * Fix Windows build
* Add TESSERACT_ENABLE_EXAMPLES compile option (`#173 <https://github.com/tesseract-robotics/tesseract_planning/issues/173>`_)
* Contributors: John Wason, Levi Armstrong, Matthew Powelson

0.9.3 (2022-02-22)
------------------
* Update changelog
* Python patches for Feb 2022 update (`#172 <https://github.com/tesseract-robotics/tesseract_planning/issues/172>`_)
* Use windows-2019 for github action
* Contributors: John Wason, Levi Armstrong

0.9.2 (2022-02-07)
------------------
* Update changelog
* Add robust method for assigning data for TOTG (`#169 <https://github.com/tesseract-robotics/tesseract_planning/issues/169>`_)
* Contributors: Levi Armstrong

0.9.1 (2022-01-27)
------------------
* Update changelog
* Update fix_state_collision_task to skip cartesian waypoints
* Contributors: Levi Armstrong

0.9.0 (2022-01-26)
------------------
* Update changelog
* Fix thread safety issue with TaskInfoContainer
* Contributors: Levi Armstrong

0.8.1 (2022-01-24)
------------------
* Update changelog
* Second attempt to fix random serialization segfault
* Contributors: Levi Armstrong

0.8.0 (2022-01-20)
------------------
* Update changelog
* Update dependency versions
* Expose save input and output for tasks in process planning request
* Simple planner should default to path profile if it exists
* Relax assert on satisfiesPositionLimits tolernace to 1e-4 for trajopt planners
* Update for fix in checkTrajectory and supporting functions
* Add seed parameter to cartesian waypoint (`#161 <https://github.com/tesseract-robotics/tesseract_planning/issues/161>`_)
* Add path profile to plan and move instruction and modify simple plan profile interface (`#159 <https://github.com/tesseract-robotics/tesseract_planning/issues/159>`_)
* Make ProcessPlanningServer::run const (`#160 <https://github.com/tesseract-robotics/tesseract_planning/issues/160>`_)
* Fix random boost serializaton segfault
* Print collision details when fix state collision task trajopt failure
* Contributors: Levi Armstrong, Matthew Powelson

0.7.3 (2021-12-21)
------------------
* Update changelog
* Add additional option to fix state collision profile
* Contributors: Levi Armstrong

0.7.2 (2021-12-16)
------------------
* Update changelog
* Add upsample trajectory task generator
* Fix bug in descartes returning solution within contact margin
* Contributors: Levi Armstrong

0.7.1 (2021-12-15)
------------------
* Update changelog
* update rosinstall and CI tags
* Only check kinematics if built in debug (`#149 <https://github.com/tesseract-robotics/tesseract_planning/issues/149>`_)
  * Only check kinematics if built in debug
  * Global process plans should not fix raster start and end position based on the global results
  * Add typeid name to failed to find profile message
  * Fix clang-tidy issues
* Fix bug in getClosestJointSolution in simple planner utils
* Contributors: Levi Armstrong

0.7.0 (2021-12-06)
------------------
* Update changelog
* Remove ACM from the descartes planner and use ContactManagerConfig
* Fix fix state collision task generator unit tests
* Update dependency versions
* Update renaming of ContactManagerConfig variables
* Add ContactManagerConfig inside CollisionCheckConfig
* Add applyCollisionCheckConfig to contact managers
* Add AllowedCollisionMatrix to CollisionCheckConfig
* Support moving AllowedCollisionMatrix into tesseract_common namespace
* Correctly set collision margin data in collision checking utils
* Contributors: Levi Armstrong, Matthew Powelson

0.6.8 (2021-12-01)
------------------
* Update changelog
* Fix bug in trajopt ifopt default plan profile
* Contributors: Levi Armstrong

0.6.7 (2021-11-30)
------------------
* Update changelog
* Update dependencies tags
* Fix trajopt ifopt composite profile check for adding collision
* Contributors: Levi Armstrong

0.6.6 (2021-11-29)
------------------
* Fix initial version in package.xml
* Add changelogs
* Update dependency tags
* Add acceleration and jerk ifopt support
* CollisionConstraintConfig set default type to DISCRETE_CONTINUOUS
* Fix ability to use same task with different parameters adding namespaces to the profile dictionary
* Update CI docker tag and target linking order (`#135 <https://github.com/tesseract-robotics/tesseract_planning/issues/135>`_)
  * Update CI docker tag
  * Update target linking order
* Remove gh_pages directory
* Contributors: Levi Armstrong, Levi-Armstrong

0.6.5 (2021-11-11 15:50)
------------------------
* Add max_steps to lvs no IK simple planner
* Contributors: Levi-Armstrong

0.6.4 (2021-11-11 12:25)
------------------------
* Fix motion planner freespace example opw_kinematics include build error (`#131 <https://github.com/tesseract-robotics/tesseract_planning/issues/131>`_)
  * Fix example opw_kinematics include build error
  * Update working_frame and tcp_frame for all motion planner examples
* Add lvs simple planner that does not use inverse kinematics and set as default
* Update readme and CI to point to tesseract-robotics (`#130 <https://github.com/tesseract-robotics/tesseract_planning/issues/130>`_)
* Contributors: Chen Bainian, Levi Armstrong, Levi-Armstrong

0.6.3 (2021-11-03)
------------------
* Update taskflows to leverage graph taskflow
* Move problem generator inside the motion planner
* Add profile dictionary to planning request
* Add contributing and license files
* Contributors: Levi-Armstrong

0.6.2 (2021-10-29)
------------------
* Make environment cache methods const (`#124 <https://github.com/tesseract-robotics/tesseract_planning/issues/124>`_)
  * Make environment cache methods const
  * Update docker version
* Update to leverage environment getGroupJointNames (`#123 <https://github.com/tesseract-robotics/tesseract_planning/issues/123>`_)
  * Update to leverage environment getGroupJointNames
  * Update CI docker tag to 0.6.2
* Contributors: Levi Armstrong

0.6.1 (2021-10-20)
------------------
* Merge pull request `#122 <https://github.com/tesseract-robotics/tesseract_planning/issues/122>`_ from marip8/update/ci
  CI Update
* Updated ccache directory environment variable
* Updated AFTER_SCRIPT for CI configuration
* Updates for windows build
* Updated install/export of motion planner targets
* Bump tesseract version to 0.6.1
* Updated CI to build on new tesseract docker images
* Contributors: Michael Ripperger

0.6.0 (2021-10-13)
------------------
* Update clang tidy CI to use noetic
* Fix clang tidy errors
* Update dependencies git hash
* Fix getRobotConfig to work with JointGroup
* Fix bug in trajopt and trajopt_ifopt problem generator
* Update tesseract_process_managers to leverage JointGroup and KinematicGroup
* Update tesseract_motion_planners to leverage JointGroup and KinematicGroup
* Update tesseract_command_language based on ManipulatorInfo change
* Update due to changes related to trajopt
* Update debug_observer.cpp to leverage console bridge
* Update based on change in trajopt ifopt (`#90 <https://github.com/tesseract-robotics/tesseract_planning/issues/90>`_)
  Co-authored-by: cbw36 <cwolfe1996@gmail.com>
* Motion Planner Package Reorganization (`#114 <https://github.com/tesseract-robotics/tesseract_planning/issues/114>`_)
  * Moved motion planners core code into core subdirectory
  * Moved simple planner into core subdirectory
  * Moved Descartes planner to new subdirectory
  * Moved OMPL planner to new subdirectory
  * Moved Trajopt planner to new subdirectory
  * Moved Trajopt IFOPT planner to new subdirectory
  * Revised main CMakeLists.txt; added options for building planner implementations
  * Reference CMake options for building tests and examples
  * Update dependencies on Descartes
  * Add dependency on tesseract collision
* Add SWIG shared_ptr to trajectory containers
* Add trajectory container class to abstract command lanaguage from time parameterization (`#44 <https://github.com/tesseract-robotics/tesseract_planning/issues/44>`_)
* Update to latest descartes and fully integrated changes with kinematic redundant solutions (`#106 <https://github.com/tesseract-robotics/tesseract_planning/issues/106>`_)
  * Update to latest descartes and fully integrated changes with kinematic redundant solutions
  * Update descartes hash in rosinstalls
  * fixup
  * Update dependencies.rosinstall
  * Update dependencies_with_ext.rosinstall
  * Update dependencies.rosinstall
  * Update dependencies.rosinstall
* clang format and fixes for CI
* Add TaskInfo statistics and custom DOT file generator
* Add serialization for TaskInfo
  Changes after review and add instructions to the serialization unit test
* Add elapsed time to TaskInfo
* Update Descartes planner (`#87 <https://github.com/tesseract-robotics/tesseract_planning/issues/87>`_)
  * Update headers and interfaces for Descartes
  * Update robot sampler
  * Add state evaluator to Descartes plan profile
  * Update .rosinstall
  * Bumped Tesseract hash in .rosinstall
  * Remove references to descartes_samplers descartes_opw
  * Update robot sampler constructor for clang-tidy
  * Added documentation to Descartes default profile
  * Remove unused Descartes utilities function
  * Move graph construction and search into try-catch block
  * Updated .rosinstall files for later Descartes dependency
  * Fixed nightly build CI configuration
* Python Fixups (`#85 <https://github.com/tesseract-robotics/tesseract_planning/issues/85>`_)
* Update rosinstall with tag/hash and create an unstable CI build using master branches (`#82 <https://github.com/tesseract-robotics/tesseract_planning/issues/82>`_)
* Allow setting planner name
* Clean up getRobotConfig and rename getRedundancy to getJointTurns
* Fix robot config redundancy calculation
* Set descrete lvs post check as default for taskflows
* Remove unused parameter from descartes default plan profile
* Fix conflict with windows macro max
* Address SWIG issues in command language
* Add missing header limits to command language utils
* Add missing console bridge header to iterative spline test
* Add missing boost header in trajopt default plan profile
* Update CI to work with taskflow v3
* Update to taskflow 3.0
* Add missing depends Threads to tesseract_motion_planners
* Remove unused class_loader includes
* Fix passing of meta information in simple planner and min_seed_length
* Switch windows build to use nmake
* Make Instruction and Waypoint default constructor private
* Switch type erasure cast methods to return references instead of pointer
* Rename Instruction and Waypoint cast and cast_const to as
* Add back NullInstruction and NullWaypoint Types
* Move serialize implementation to cpp based on boost documentation for shared libraries
* Remove NullWaypoint and NullInstruction types
* Switch over command language to using boost serialization
* Update descartes planner to use default values for satisfiesPositionLimits
* Reduce limits slightly in FixStateBoundsTaskGenerator
* Enable OMPL to plan for paths with more than one instruction (`#49 <https://github.com/tesseract-robotics/tesseract_planning/issues/49>`_)
  * Enable OMPL to plan for paths with more than one instruction
  * Rebase on latest and update unit test
  Co-authored-by: Levi Armstrong <levi.armstrong@swri.org>
* Enforce bounds on seed pulled from environment current state
* Increase descartes assert epsilon for satisfiesPositionLimits check
* Run enforce bounds on result trajectory for all motion planners
* Check start and goal bounds
* Add SetAnalogInstruction
* Correctly populate start instruction velocities in TOTG
* Fix ProfileDictionary use and profile entries in Python
* FixStateCollision: Only adjust the states that are in collision
* Update due to changes with CollisionMarginData
* Update to use boost targets (`#46 <https://github.com/tesseract-robotics/tesseract_planning/issues/46>`_)
* Switch to using Eigen target
* Add Set Tool Instruction
* Update descartes and ompl to leverage CollisionCheckConfig
* Add profile overrides to Move, Plan, and Composite Instructions
* Fix passing of meta information through TOTG
  Note that it will still be partially lost if it change in the middle of a sub-composite.
* Add sub-composite rescaling to TOTG task generator
* Allow nonconditional nodes to be attached to arbitrary nodes
* Allow graph taskflows that connect non-leaf nodes to error/done
* Fix misc typos
* Fix toDelimitedFile unit test
* Fix compiler error for boost::is_virtual_base_of for versions prior to 1.67 (pagmo)
* Add boost serialization for the command language along with unit tests
* Add Missing Include Statement
* Update robot config with new kinematics interface
* Add the ability to save the Task inputs and outputs to the TaskInfo
* Update to new forward and inverse kinematics interface
* Merge pull request `#36 <https://github.com/tesseract-robotics/tesseract_planning/issues/36>`_ from mpowelson/feat/tolerance_xml
  Add Tolerances to joint/cartesian waypoint XML
* Template serialize/deserialize functions and add waypoint unit tests
* Add Tolerances to joint/cartesian waypoint XML
* Add operator == to CartesianWaypoint and JointWaypoint
* Update to latest tesseract_environment changes
* Fix divide by zero case in time optimal trajectory generation
* Add joint waypoint isToleranced unit test
* Remove tcmalloc as a dependency in package.xml and cmake config.in
* Add link directories for ompl to support windows
* Use almostEqualRelativeAndAbs for checking if tolerances were provided
* Fix unitialized data in time optimal time parameterization and start to fixing float equal comparison
* Update readme to include ptmalloc documentation and update tcmalloc documentation
* Fix bug in task info
  The name was getting stored in message instead of task_name
* Add workaround for TOTG failure on duplicate points
* Update cmake_common_scripts to ros_industrial_cmake_boilerplate
* Update to latest descartes_light
* Remove tcmalloc as a library dependency. tcmalloc should only be used when liking an executable.
* Add time optimal trajectory generation TOTG (`#23 <https://github.com/tesseract-robotics/tesseract_planning/issues/23>`_)
* Add tesseract_command_langauge package from tesseract repo
* Fixes to trajopt_ifopt planner
* Fix toleranced waypoints being added as "fixed" in trajopt planner
* Fix passing tolerances through Trajopt planner
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
* Update README.md
* Remove setters and getters and make variable public in simple planner profiles
* Move step generator code into the profile classes
* Improve simple planner profiles to handle working frame
* Add documentation about libc tunables (`#5 <https://github.com/tesseract-robotics/tesseract_planning/issues/5>`_)
* Change rosinstall to use specific versions of external dependencies
* Update CI
* Add back ci, docs, etc. after split
* Update packages package.xml to include buildtool_depend on cmake and exec_depend on catkin
* Move tesseract_command_language out of tesseract_planning directory
* Switch DebugObserver to use console bridge
* Add error task and done task to GraphTaskflow
* Add TrajOpt Ifopt planner (`#443 <https://github.com/tesseract-robotics/tesseract_planning/issues/443>`_)
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
* Update planners to use CollisionCheckConfig
* Update motion planners to account for Joint and State Waypoints unordered joints relative to kinematics
* Add support for external tcp attached to kinematic link
* Utilize  parameter in TrajOpt planner
* Update to use initialize_code_coverage() macro and compiler definition
* Extract package name and version from package.xml
* Get Robot Redundancy (`#486 <https://github.com/tesseract-robotics/tesseract_planning/issues/486>`_)
  Co-authored-by: Colin Lewis <ctlewis@swri.org>
  Co-authored-by: David Merz, Jr <david.merz@swri.org>
* Remove tesseract_process_planners package
* Fix message in default ompl plan profile
* Update to clang-tidy version 10
* Make non-virtual-dtor errors
* Remove process_managers, replaced by planning server
* Remove deprecated collision class methods and utility functions
* Python package updates for command language
* Add missing colcon.pkg files
* Simplify the process generator interface to avoid std::function
* Make changes to better support python wrapping
* Remove tesseract package
* Add external tool center point support
* Add generateNaiveSeedGenerator function
* Add TrajOpt Solver Profile
* Clean up warnings related to setContactDistanceThreshold
* Fix bug in createCollisionTermInfo
* Add core directory to tesseract_process_managers
* Update ProfileDictionary and add additional unit tests
* Update state sampler allocator function signature
* Switch to using lambda over std::bind and remove NOLINT
* Add bool has_seed to ProcessInput and add back GraphTaskflow
* More documentation, remove commented code, some requested changes
* Add doxygen and a few bug fixes
* Make profiles and ProfileDictionary const
* Fix issue in lvs cart cart interpolation
* Restructure taskflow generators to support composition
* Make trajopt, ompl, descartes, freespace and cartesian taskflow generators
* Fix enabling of simple planner
* Move default process planners to method that user calls
* Add profile dictionary
* Create process planning server
* Add feedback of contacts to FixStateCollisionProcessGenerator
* Add virtual destructor to ProcessInfo as well as bug fixes
* Add SFINAE function signature check to command language
* Add SFINAE utils
* Improve error handling in joint and state waypoint
* Add ProcessInfo to process generators (`#450 <https://github.com/tesseract-robotics/tesseract_planning/issues/450>`_)
* Add wait and timer instruction to command language
* Add CollisionCheckConfig
* Fix bug in simple planner not resetting start waypoint
* fix lvs process flow and step calculation
* Add clone method to moiton planner base class
* Add vertex evaluator to descartes
* Code Simplification in StateInCollision
* Fix constraint from error function in trajopt plan profile
* Move ManipulatorManager into Environment
* Add MoveWaypointFromCollisionRandomSampler to FixStateCollisionProcess (`#426 <https://github.com/tesseract-robotics/tesseract_planning/issues/426>`_)
  * Add MoveWaypointFromCollisionRandomSampler to FixStateCollisionProcess
  * Add more generalized way of specifying correction methods
  * Bug fix
  * Add assert to catch mismatched sizes
  * Rebase fixes and a bug fix
* Add seed min length process generator and unit tests
* Update unit tests and fix lvs_interpolation
* Updated uses of fixed size interpolation to lvs interpolation
* Updated lvs tests to be more thorough
* Fix bug in trajopt default problem generator not getting composite profile correctly
* Add verbose options to process input so planner verbosity can be turned on
* Fix issue in freespace taskflow for the trajopt first condition
* Add raster only process managers
* Fix SimplePlanner step generators to correctly set profile
* Add ProfileSwitchProcessGenerator
  This generator simply returns a value specified in the composite profile. This can be used to switch execution based on the profile
* Add utility for getting profiles (`#412 <https://github.com/tesseract-robotics/tesseract_planning/issues/412>`_)
* Add unit test for generateSkeletonSeed
* Enable tesseract_motion_planners build on windows
* Address console bridge issue `#91 <https://github.com/tesseract-robotics/tesseract_planning/issues/91>`_
* Fix to handle console_bridge target renaming in noetic
* Separate public and private compiler option and add back -mno-avx
* Add individual CI badges and Windows CI build
* Check validity of longest valid segment
* Set active links based on ManipulatorInfo in contact check processes
* Add visibility control to all packages
* Expose transpose method for Joint Waypoint
* Add print to waypoint
* Update due to changes in descartes compound edge evaluator
* Fix done and error callback in simple process manager
* Remove inheritance of Eigen::VectorXd from Joint Waypoint
* Remove inheritance of Eigen::Isometry3d from Cartesian Waypoint
* Remove inheritance of std::vector from Composite Instruction
* Rename buffer_margin to safety_margin for consistency
* Change Tesseract findTCP to throw exception when not found and update planners to handle this exception
* Improve global raster taskflow
* Switch from Cast Continuous to Discrete Continuous
* Update default longest valid segment length
* Add taskflow debug and profile observer
* Improve trajectory player and add utility getJointNames from waypoint
* Fix ompl default plan profile not setting planning time
* Update CompositeInstruction toXML so Null StartInstructions are not output
* Change freespace taskflow to still try trajopt if ompl fails
* Fix graph taskflow handeling of TASK type
* Add isIdentical for two vectors of strings
* Fix descartes handeling of freespace plan types
* Add simple planner longest valid segment interpolation (`#385 <https://github.com/tesseract-robotics/tesseract_planning/issues/385>`_)
  Co-authored-by: Stevie Dale <steven.dale@swri.org>
* Change ProcessInput to better support changing data structure throughout the taskflow
* Update REP and ROP Kinematics along with ManipulatorInfo TCP support
* Add global raster variant
* Improve ignition material conversion
* Add manipulator manager to support tracking changes in the future
* Refactor fix state bounds utils to eliminate repetitive inform msgs
* Pass verbose to motion planners only when debug messages enabled
* Add clang static analyzers
* Leverage cmake_common_scripts
* Add fixStateBoundsProcessGenerator
* Clean up tesseract_process_managers and tesseract_motion_planners package
* Add fixStateBoundsProcessGenerator
* Add clampToJointLimits utility
* Switch ISP to use MoveInstructions instead of PlanInstructions
* Add motion planner serialization (`#356 <https://github.com/tesseract-robotics/tesseract_planning/issues/356>`_)
* Add Profiles to ISP Time parameterization process generator
* Allow ISP scaling factors to be changed on a point by point basis
* Process managers: Only print "Generating Taskflow for..." when log debug
* Split command_language_utils into multiple files
* Simplify raster example program
* Add simple process manager and planner profile mapping
* Add BiTRRT Configurator
* Break up serialization and deserialization and make deserialization more flexible
* Add XML serialization to tesseract_command_language
* Expose velocity and acceleration scaling factors in process generators
* Add debugging information when planning fails due to collisions
* Change OMPL default safety margin to 0.0
  This essentially removes the 0.025 inflation that was added previously.
* Fix typo in ISP ProcessGenerator
* Fix Clang Tidy errors
* Fix/Add clearing of graph and sequential taskflow
* Add graph taskflow
* Fix const and indexing issue in tesseract planning
* Remove unused examples and dependencies from tesseract_command_language
* Add iterative spline parameterization process generator
* Add support for velocity and acceleration limits
* Rename iterative spline parameterization methods
* Add tesseract_time_parameterization package include iterative spline algorithm
* Remove random generators and validators
* Add discrete and continuous process generators
* remove dependency descartes_opw
* Add new JointWaypoint constructor and fix clang tidy errors
* Switch to using unique pointer for Process Generator
* Rename sequential_failure_tree_taskflow to sequential_taskflow
* Make command language utility function generic and move planner specific ones to motion planners package
* Get tesseract process managers working
* Swith process input to leverage pointer instead of references
* Improve support for state waypoint in simple motion planner
* Update tesseract_process_managers
* Update tesseract_command_language and tesseract_motion_planners
* Update/Fix tesseract process manager
* Make requested changes
* Remove unused header from motion planning example
* Add unit tests for fixed size assign position
* Update/Add examples to leverage ignition visualization
* Update motion planners to leverage new flatten utils and non-const getWaypoint
* Fix flatten utils and add non-const getWaypoint for Move and Plan Instruction
* Address requested changes
* Add missing include <atomic>
* Add missing SHARED to libraries
* Add skeleton unit test for fixed size assign position
* Update motion planner example
* Address todo's in tesseract_motion_planners
* Fix simple planner fixed size interpolate unit tests
* Handle multple solutions in fixed_size_interpolate.cpp
* Fix motion planners unit tests
* Bring back generateSeed, add readme, and add task validators
* tesseract_motion_planners: Alphabetize CMake targets
* Add SimpleMotionPlanner
  The simple planner is meant to be a tool for assigning values to the seed. The planner simply loops over all of the PlanInstructions and then calls the appropriate function from the profile. These functions do not depend on the seed, so this may be used to initialize the seed appropriately using e.g. linear interpolation.
* Replace position, velocity, etc in MoveInstruction with StateWaypoint
  This will allow us to change what the results of planners are without changing the MoveInstruction interface
* Add ManipulatorInfo to PlanInstruction
* Misc improvements and rebase fixes
  Modify examples so the complete successfully and clean some things
* Update Defaults and add ability to abort process
* Add OMPL and Descartes support
* Update start and end Instructions in process managers
* tesseract_process_managers: Add raster_process_manager
  Adds the groundwork for a raster process manager along with an example using random processes.
* tesseract_process_managers initial commit
* Add missing include
* Add simple motion planning example using command language
* Fix ompl planner unit test
* Add missing license and warnings macro to files
* Switch setStartWaypoint to setStartInstruciton and update planners
* Fix descartes processing of results to handle freespace correctly
* Fix use of flatten functions and fix trajopt problem generator
* Tesseract_planning: Add data to request/response
* Clang Tidy fixes
* Move Flatten Utilities into tesseract_command_language
* Add option to include composites in results when flattening
* Tesseract planners: Make solve method const
* Simplify instruction class signature and utility functions
* Clang format
* Descartes planner: Copy solution into response
* Fix motion planner unit tests
* Add command language utils
* Fix trajopt and descartes missed merge issues
* Bug Fixes
* Refactor OMPL to use request/response
* Refactor Descartes to use request/response
* Refactor TrajOpt to use request/response
* Add command_language.h
* Change how start waypoint is defined, now provided by CompositeInstruction
* Clang format
* Update OMPL planner to support cartesian waypoints and supporting unit tests
* Remove hybrid planners
* Add Flatten utility
* Add basic print functions to instructions
* Improve descartes collision edge evaluator unit run time
* Clang-Format
* Update ompl to use new kinematics objects and fix clang-tidy
* Update descartes to only use new tesseract_kinematics objects
* Update OMPL to leverage command language
* Remove hybrid planners
* Working descartes unit tests with command language
* Working trajopt unit tests with command language
* Update generateSeed utility function for linear
* First pass at updating tesseract_motion_planners unit test with command language
* Fix error in isJointWaypoint
* Move new planner profiles to tesseract_motion_planners
* Switch to using profiles for plan instructions and composite instructions
* Add tesseract_command_language package
* Added fixed timesteps to TrajOpt config
* Setting Active collision objects for the contact managers in trajopt motion planner
* Bugfix when OMPL simplifies down to two states and trajopt was assuming > 2, so segfaulting
* Add COLCON environment hooks to update ROS_PACKAGE_PATH
* Add Noetic CI Build (`#305 <https://github.com/tesseract-robotics/tesseract_planning/issues/305>`_)
  * Add Noetic CI build
  * remove redundant move
  * Add missing static_casts
  * remove more redundant moves
  * Another redundant move
  * Remove old header.
  * Add Python 3.8
  * Add Colcon environment hooks for Python packages
  * Bump tesseract_viewer_python required cmake version to 3.5.0
  * Add python version to tesseract_viewer_python
  * Source workspace before testing
  * Remove after script
  The tests are being run by colcon anyway
  Co-authored-by: Matthew Powelson <powelson.matthew@gmail.com>
* Add colcon.pkg files to all packages (`#303 <https://github.com/tesseract-robotics/tesseract_planning/issues/303>`_)
  * Add colcon.pkg files to all packages
  Addresses issue `#302 <https://github.com/tesseract-robotics/tesseract_planning/issues/302>`_ as discussed on rosdep issue 724.
  * tesseract_collision: Remove pluginlib workaround
  This is now handled in the tesseract_configure_package macro
  * Add benchmark to the xenial nightly build skip keys
* Rewrite of the srdf model class within tesseract (`#292 <https://github.com/tesseract-robotics/tesseract_planning/issues/292>`_)
  * Clean up SRDFModel and restructure
  * Add opw kinematic parsing to srdfmodel and update tesseract python
  * Fix SWIG Python data types in srdf_model.i
  * Add new construction method to joint waypoint type
  * Move SRDFModel OPWKinematicsParameters structure outside the class
  * Fix SWIG build error in sdf_model.i
  * Clang format and fix random number definition
  * Remove unsupported methods in TinyXML2 in Kinetic
  * Expose resource locator in tesseract object
  * Modify collision large dataset unit to print information
  * Break up srdf_model.cpp into smaller files and fix requested changes
  Co-authored-by: John Wason <wason@wasontech.com>
* add process definition generator interface (`#289 <https://github.com/tesseract-robotics/tesseract_planning/issues/289>`_)
  * add process definition generator interface
  * Created Doxygen, removed constructor, and standardized compiler def
  * clang'd
  Co-authored-by: ctlewis <colin.lewis@swri.org>
* Add ability to provided IsContactResultValid function in contact request.
  * Added special collision pairs for trajopt planner
  * Added capablities to allow negative special collision pairs to pass post-check
  * Removed commented code
  * Clang formatting
  * Removed unnecessary lines
  * Removed unused variables
  * Changed collision pairs to use safetyMarginData type
  * changed nullptr assignment
  * Moved negative collision checking into trajectoryValid function
  * Clang formatting
  * Fixed build test failing
  * Clean up
  * Fix clang-tidy errors
  Co-authored-by: Tyler Marr <tyler.marr@swri.org>
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Store joint transforms in EnvState structure (`#265 <https://github.com/tesseract-robotics/tesseract_planning/issues/265>`_)
* Explictily instantiate Descartes Hybrid planner
  Indeed, this template is defined in a .cpp so it needs to have explicit
  instantiation, done for double and float
* Fix error message for samplers in Descartes
  It used to say that the number of waypoints was wrong
* Add code coverage to packages tesseract, tesseract_process_planners and tesseract_visualization
* Add code coverage macros and add code coverage to packages
* Configurable post-plan collision check (`#247 <https://github.com/tesseract-robotics/tesseract_planning/issues/247>`_)
  * Added trajectory validator class
  * Updated planner base class to use trajectory validator class
  * Updated planners to use trajectory validator class
  * Updated python interface
  * Updated OMPL TrajOpt unit test
  * Clang format
* Disable ompl trajopt hybrid unit test
* Adjust ompl trajopt hybrid unit test
* Modify hybrid ompl trajopt planner to set range on ompl planner
* Fix ompl kinetic unit tests
* Remove additional planners from the ompl unit tests
* Adjust ompl unit tests and add asserts
* Only add state collision validator when continuous_collision is false
* Add ompl glass up right example
* Move ompl constrained to its own config
* Use ompl state extractor to eigen and add state validator
* Update to use generic method for extracting data out of ompl state
* Add ability to add constraints to ompl planner
* Remove OMPL EST planner from the unit tests
* Add Extension Approach Generator
* Add end state to the process definition structure
* Adjust Departure Generator (`#228 <https://github.com/tesseract-robotics/tesseract_planning/issues/228>`_)
  * Adjust Departure Generator
  * Moving extension departure generator to separate file
  * Removing Whitespace to Appease Clang
  * Adding License to Extension Departure Generator
  * Adding License Text to tesseract_planning Files
  * Adding @briefs to the comment blocks at head of tesseract_planning files
* Update ompl trajopt hybrid test to only add collision as a constraint
* Use the ompl seed trajectory to set trajopt num_steps in hybrid planner
* Fix ompl unit tests
* Trajopt Planner: Set init data when using JOINT_INTERPOLATED
  Currently JOINT_INTERPOLATED is unusable since the data is not set.
* Add JOINT_WAYPOINT to fixed_steps list only if it isCritical
  Currently it treats any joint position waypoint as fixed which may not be the case depending on the coefficient
* Modify OMPL planner and config to accept multiple planner types
* Add ability to merge a SceneGraph into another one (`#219 <https://github.com/tesseract-robotics/tesseract_planning/issues/219>`_)
  * Allow to merge a SceneGraph into another one
  Needed to create prefixed copy operators for links and joints
  * Delete Link & Joint copy constructor / assignment
  This means a large refactoring of the codebase to remove all instances
  - Add some functions that take a Ptr as argument, to avoid having to
  move instances being pointed at
  - Add calls to std::move where appropriate
  - Modify the code to no longer use moved instances
  * Use std::move in tesseract_rosutils
  * Use std::move in tesseract_scene_graph unit tests
  * Use std::move in tesseract_motion_planners
  * Use std::move in tesseract_rviz
  * Use std::move in tesseract_examples
  * Update tesseract_python to support move semantics
  This requires the introduction of 3 changes:
  - In scene_graph, only bind Ptr versions
  - In environment, introduce custom wrappers that copy the incoming Ptr
  - In msg conversions, use a new macro type that moves the return value
  into a Ptr
  * Fix the clang-tidy warnings
  * Make adding of joints / links pointers protected
  This ensures that nobody can modify the scene graph once built
  This required a tiny hack in the URDF parser, we should upgrade the
  interface to unique pointers in the future.
  * Update documentation for addSceneGraph
  * Make name\_ a non-const member of Joint and Link
  * Fix tesseract_python to clone the links
  They can only be passed by pointer
  * Wrap <queue> include with ignore warnings macros
  * Use variables for joint & link names in tests
  This only concerns tesseract_environment_unit for now
  * Fix test: was using link after moving it
  Created a variable to hold the name, and use that instead of getName()
* Add eigen to package.xml
  and alphabetize the entries.
* Expose trajopt collision term use_weighted_sum
* Updated axial approach and departure generators to interpolate orientation and to not duplicate the starting point
* Set collision cost safety margin buffer to zero by default
* Add safety_margin_buffer fields to Tesseract Trajopt planner objects
* Fix missed disabling ompl planner hybirdization when config param optimize set to true
* Restructure ompl to leverage config structures like the trajopt planner
* Add optimization capability for OMPL freespace planner
* Allow adding TrajOpt collision terms as both constraints and costs (`#210 <https://github.com/tesseract-robotics/tesseract_planning/issues/210>`_)
  * Add separate collisions terms for constraint and cost and expose in planner config
  * Add config structs for collision costs and constraints
  * Use 'enabled' instead of 'check'
  * Add missing license block
  * Clang format
  * Fix typo in license
  * Add swig wrapper for trajopt_collision_config.h
  * Add collision config members to Swig wrapper for default planner config
  * Fix collision enable/disable in tests
  * Update collision constraint def to new format
* Fix bug in descartes robot positioner sampler storing positioner limits as wrong type
* Change Eigen arguments that are passed by value to reference
* Fix bug in trajopt default config accessing nullptr
* Add useful operators to Joint and Cartesian Waypoints
* Make requested changes
* Update ompl freespace planner to use Parallel Plan with hybridization disabled
* Remove descrete collision check from ompl continuous motion validator
* Update to support trajopt new discrete continuous
* Update trajopt planner handling of fixed start and end states for collision
* Turned avoid singularity off by default
* Changed planner debug logging from debug to info
* Clang tidy updates
* Changed default waypoint constraint names
* Added avoid singularity to TrajOpt motion planner utils and default configuration
* Update due to changes in TrajOpt CollisionTerm supporting longest valid segment length
* Update motion planners post check to only use continuous contact checking
* Switch to using state solver in descartes edge evaluator and ompl motion validator
* Add descartes collision edge evaluator to descartes unit tests
* Update dates and add asserts
* Clang Formatting
* Add descartes collision edge evaluator
* Update checkTrajectory and supporting funtion to state solver and contact test type
* added the default constructor to the ProcessPlanner class
* Add doxygen comment to contact_dist_threshold\_ member
* Add parameter to set DescartesCollision contact distance threshold
* Add processing of header files to clang-tidy
* Change how unit test are ran
* Set trajopt log level to Error to limit CI error log to long
* Fix ompl to obey collision safety margin
* Improve checkTrajectory, OMPL and TrajOpt planners by adding longest_valid_segment_fraction and longest_valid_segment_length
* Clang format
* Fix ompl planner response and verify final trajectory is collision free
* Address remaining compiler and clang tidy warnings
* Improve ompl handling of the number of output states
* Expose ability to set collision coeff in trajopt configs
* Add ability to add user defined trajopt constraint type and coeff
* Update based on Clang-Tidy
* Update based on Clang-Tidy and Clazy
* Fix issue with descartes pose sampler
* Update ompl trajopt hybrid planner to use new resoure locator api
* Use ResourceLocator class instead of ResourceLocatorFn (`#172 <https://github.com/tesseract-robotics/tesseract_planning/issues/172>`_)
  * Use Resource and ResourceLocator instead of locateResource function
  * More updates to use ResourceLocator
  * More updates to use ResourceLocator
  * Fix clang-format
  * Update Resource and ResourceLocator to use ROS Cpp style guidelines
  * Fix comments in resource_locator.cpp
  * Improve doxygen comments in resource.h and resource_locator.h
  * Clang format
* Added license to OMPL hybrid planner
* Added unit test for OMPL TrajOpt planner
* Added OMPL hybrid planner
* Add check in trajopt config for start joint waypoint not matching seed trajectory start
* Adjust for joint waypoint joint name order
* Add name to tesseract trajopt planner constraint from error function
* Update trajopt planner to use trajopt UserDefinedTermInfo for error functions
* OMPL Planner Simplification (`#160 <https://github.com/tesseract-robotics/tesseract_planning/issues/160>`_)
  * Updated OMPL config structure
  * Updated OMPL unit to use typed test to test all OMPL planners
  * Clang format
  * Reorganized collision checking logic
  * Added optional interpolation parameter to OMPL config
  * Turned off continuous collision checking, added interpolation, and increased planning time in OMPL test
* Process Planner: Generating add from_start to generateProcessDefinition
* Trajopt Planner: Switch setConfiguration to pass shared_ptr by value
  When passed by reference, calling clear on the planner also clears the config that was passed in. If it is by reference, you will just be setting the planners config to nullptr not the original.
* Exposes joint weighting in trajopt default configuration
* Add iterators to process segment definition class
* Trajopt Planner: Expose QP Solver selection
* Allow is_valid nullptr for descartes samplers
* Fix casting of float array to Eigen VectorXd in descartes_collision.hpp
* Add constraint from error function option to the trajopt default config
* Add cmake macros to simplify cmake files
* Use GTest named targets instead of lib and include
  ${GTEST_BOTH_LIBRARIES} becomes GTest::GTest and GTest::Main
  GTEST_INCLUDE_DIRS is no longer needed
* Rename class and document new code
* Add descartes sampler for a single manipulator
* Switch to using descartes samplers for railed and positioner systems
* Add descartes collision, railed kinematics and positioner kinematics
* Updated planner inheritance; added licenses; changed header include symbols
* Merged TrajOpt planner config base with planner config
* Clang formatting
* Added check for joint waypoint in first or last position for default TrajOpt planner config
* Updated trajopt motion planner test
* Updated Descartes hybrid planner with new configuration classes
* Removed TrajOpt array and freespace planners
* Created TrajOpt configuration classes and utilities
* Created TrajOpt planner configuration abstract base class with a method to create a TrajOptProb. Updated the TrajOpt motion planner to utilize the base configuration class
* delete unused #include <ros/console.h>
* Add ctest output log
* Fix ctest verbose output
* Updated TrajOpt planner unit test
* Clang formatting
* Updates to generators and examples to utilize cartesian pose getParentTransform method
* Updated Cartesian waypoint to hold a link relative to which its transformation is relative
* Updated joint toleranced waypoint to inherit from joint waypoint
* Clean up urdfdom references
* Add AVX warning when compiling with non-GNU compiler
* Add -mno-avx as compile option to fix Eigen Alignment Issues
* Descartes_tesseract_kinematics: Add license and harmonizeTowardsZero
* Add DescartesTesseractKinematics wrapper
  This adds a wrapper for a TesseractKinematics object such that it can be used with Descartes. It has currently only been tested with the default KDL kinematics
* Add addition doxygen, unit tests, and clang format  addressing PR comments
* Add discrete checking to ompl continuous motion validator to catch self collisions
* clang format
* Add descartes motion planner unit test
* Add num_threads to descartes config and remove use of ROS_ERROR for descartes planner
* ompl freespace check start and end position for collision
* Add isValid check in continuous and discrete motion validators
* Add discrete motion validator and cache contact managers in validators
* Update ompl freespace planner to use OMPL OptimizePlan
* Fix compiler warnings in waypoint.h
* Add license to new ompl files and add doxygen
* Add ompl planner specific setting and fix naming
* Switch to use Valid State Sampler to avoid cloning contact manager for every isValid check
* Update OMPL planner to planner interface
* Clang format
* Changed logic to fail if optimizer does not converge
* Updated planners to implement changed in base class solve method
* Updated motion planner base class solve method to take optional verbosity argument
* Add succeeded waypoints and failed waypoints to PlannerResponse
* clang format
* Update to allow null collision interface for descartes planner
* Fix descartes config struct
* Add JointTrajectory structure
* Update waypoint types with constructors and setters and getters
* Clean up descartes planner
* Rename process planner to_start to to_end
* Update planners to use status code and add descartes planner and descartes-trajopt hybrid planner
* Correct planners to fill out response when not configured
* Add TrajOpt Planner unit tests
  These tests test the TrajOptArrayPlanner and the TrajOptFreespacePlanner. They primarily check that the correct types
  of costs and constraints are added when the flags like smooth_velocity are specified. However they are not foolproof.
  They only check that at least one term of the correct type is in the cost or constraint vector. If there should be
  more than one, then it might not be caught. This could be improved in the future, but it is better than nothing.
  Additional features that could be tested in the future
  * Configuration costs added correctly
  * Intermediate waypoints added correctly to freespace
  * coeffs set correctly
  * init info is set correctly
  * Seed trajectory is set correctly
  * callbacks are added correctly
  * Number of steps are obeyed for freespace
  * continuous collision checking flag set correctly
* Add dependencies for tests on package libraries
* Fix clang warnings
* Update rosdep keys in package.xml
* Clange format version 8
* Unify shared pointer definition and switch typedef to using
* Create process planning package (`#16 <https://github.com/tesseract-robotics/tesseract_planning/issues/16>`_)
  * added the tesseract_process_planning package
  * added the conversions header to the tesseract_rosutils package
  * added the improvements made by @mpowelson to the process definition methods
  * renamed tesseract_process_planning to ...planners for consistency
  * reinstated previous dependencies in cmake file
  * corrected namespaces and header guards
  * renamed some directories in tesseract_process_planners and documented a base class
  * renamed tesseract_planners to tesseract_motion_planners
  * renamed the base class BasicPlanner to MotionPlanner
  * renamed from_home field to from_start
  * improvements to the tesseract MotionPlanner interface and trajopt derived classes
  * removed the const attribute from all the solve(...) methods
  * Clean up cmake add missed renaming from tesseract_planners to tesseract_motion_planners
* Post checks trajectories in trajopt_planner for collisions (`#15 <https://github.com/tesseract-robotics/tesseract_planning/issues/15>`_)
  * Add checkTrajectory for discrete collision check
  * Add discrete collision check to trajopt_planner
  And return false if error code is negative
  * Fix checkTrajectory
* Update unit tests
* Move contents of tesseract_planning into tesseract_planners
* Namespace targets
* Add tesseract_common package
* remove duplicate target_compile_feature
* Improve free space planner to not add costs and aux constraints on fix joint waypoints
* Fix kinetic c++11 cmake flag
* Fix interface targets method of adding c++11 compiler option
* Add cmake support for xenial builds
* Header and namespace bug fixes
* Update to changes in trajopt problem description use of tesseract
* Fix ompl planner test to use tesseract class
* Add tesseract class
* Rename tesseract_core folder to tesseract
* Fix tesseract_rviz trajectory visualizaiton when start state is not visible
* Update rviz widgets due to changes in tesseract
* Update rosutils parsing changed messages
* Add command history to tesseract state message
* Add scale variable to Mesh message
* Get tesseract trajopt examples working
* Add scale meta data when parsing meshes
* Add untility function to calculate jacobian numerically and update test to use
* Add scale meta data to meshes loaded from file
* Make getCommandHistory const and intialize member variables
* Fix basic cartesian plan example
* Minor fixes in tesseract_rviz render tools
* Fix tesseract_environment to get/set name from scene graph
* Switch to using gtest_discover_tests versus add_tests
* Update unit tests and first pass at updating examples
* Seperate the environment state solver into its own class to simplify creation of new environment classes
* Simplify tesseract_rviz and update to work with new version of tesseract
* Fix naming of environment commands
* Update the environment monitor to leverate services for updating environment and getting updates from the environment
* Fix invalid warning generated from tesseract_rviz EnvVisualization
* Improve tesseract_rviz environment visualization by cloning entities for trajectory visualization
* Add command history to tesseract environment to be leverage by distributed systems
* Configure bullet to be used in a multi threaded environment
* Add contact manager factory and move allowed collision matrix to tesseract_scene_graph
* Move trajopt examples to tesseract_ros and update to new version
* Fix tesseract_rosutils plotting.h build failure
* Fix environment creation, missing allowed collision matrix
* Fix kdl parser
* Update checkTrajectory and unit test for tesseract_environment
* Make ompl planner test an gtest
* Convert tesseract_planning to pure cmake
* Update unit tests with changes in tesseract scene graph
* Get tesseract_rvis updated with changes to tesseract core packages
* WIP: Move ROS package into sub folder
* Remove use of Eigen3::Eigen target due to bug in catkin
* Improve package config files
* move tesseract_planning new file to correct location after rebase
* Add tesseract_support package for containing files for tests and examples
* Rename fcl_ros and libccd_ros packages
* Make bullet3 a pure cmake package
* Make tesseract_core pure cmake packages
* Add gtest source to tesseract_ext for pure cmake packages
* Remove tesseract_ext metapackage
* Fix getLinkChildrenNames function and add unit tests
* Add getActiveLinkNames to forward kinematics
* Add ability to enable and disable collision for link through environment api
* Add getLinkChildrenNames and getJointChildrenNames to scene graph
* First pass clean up for tesseract_ros
* Add additional check to tesseract_scene_graph unit test
* Enable ability to add and remove joints in tesseract_environment
* Add ability to clone contact manager as empty
* Fix removing of link and joint in tesseract_scene_graph
* Add SRDFModel and parser
* Add getInvAdjacencyLinks method
* Fix urdf parser of calibration tag
* Add createKinematicsMap from SRDFModel
* Add getAdjacencyMap, Remove AllowedCollisionMatrix, Force Convex Hull for Collision Objects
* add missing export dependency to tesseract_collision
* Fix tesseract_visualization include directory structure
* Move tesseract_core test to tesseract_environment
* Switch mesh face storage from std::vector<int> to Eigen::VectorXi>
* Remove tesseract_core package
* Add package tesseract_visualization
* Move tesseract_scene_graph under tesseract_core
* Move tesseract_planning under tesseract_core
* Move tesseract_kinematics under tesseract_core
* Move tesseract_geometry under tesseract_core
* Move tesseract_environment under tesseract_core
* Move tesseract_collision under tesseract_core
* Add getAdjacentLinkNames to tesseract_scene_graph
* Add tesseract_environment package
* Add moveJoint method to scene graph
* Remove export library from tesseract_collision that does not exist
* Make minor fixes to tesseract_scene_graph
* Update tesseract_kinematics to leverage tesseract_scene_graph
* Add parser for SceneGraph to KDL Tree
* Expose graph getVertex and getEdge method
* Add urdf parser and tests to tesseract_scene_graph
* Add missing include to tesseract_geometry/types.h
* Update mesh parser to include convex hulls and add unit tests
* Add asserts to mesh.h sdf_mesh.h to error if not triangle meshes
* Fix incorrect includes in cylinder.h and octree.h
* Add mesh parser test
* Add mesh parser to tesseract_scene_graph
* Add macros.h
* remove unused dependencies
* Add tesseract_scene_graph
* Add tesseract_geometry package and update tesseract_collision to leverage new package
* Create tesseract_environment and semi-isolate
* Fix depends in tesseract_kinematics
* Add meshes to tesseract_ros to remove dependency on trajopt_examples
* Semi-Isolate Tesseract Kinematics
* Make minor fixes in tesseract_collision
* Update create_convex_hull to not use ros
* Switch to using console bridge
* Isolate tesseract_collision namespace
* Switch to using built in Collision Shapes
* Add passthrough for coeffs when interpolating waypoints
  It uses the coeffs and is_critical from the starting waypoint. This was a fairly arbitrary choice.
* Fix trajopt planner to return correct status
* Fix Clang Warnings and Clang Format
* Tesseract Planner: Add Desired Configuration parameter
* Tesseract Planner: Add JointTolerancedWaypoint
  And update the Trajopt Freespace and Trajopt Array planners to use them. Note: This API is probably not set in stone at this point.
* Add utils to interpolate between cartesian poses
* Tesseract Planner: Add flag to allow switching b/n costs/constraints
* Add coeffs to Tesseract Planner Waypoints
  Used to weight different terms in the waypoint. For example: joint 1 vs joint 2 of the same waypoint or waypoint 1 vs waypoint 2
* Add ability to add tcp to array and freespace planners
* Fix clang format for tesseract_planning
* Add missing break statements in freespace planner
* Fix waypoint orientation return order and change joint waypoint to eigen
* Flip  quaternion orientation order (xyzw -> wxyz)
* Switch WaypointType to enum class, doxygen, and add ConstPtr typedefs
* Bug Fixes and Cleanup
* Add TrajOpt Array Planner
  This takes an array of points and plans through them. This could correspond to a PoseArray msg in ROS. The idea is that this planner could be used to plan one stroke in a raster path
* Add TrajOptFreespacePlanner
  This planner is intended to provide an easy to use interface to TrajOpt for freespace planning. It is made to take a start and end point and automate the generation of the TrajOpt problem. It does not expose all of the configuration available in TrajOpt, but is made in hopes of lowering the barrier to entry for using TrajOpt to solve simple problems.
* Clang formatting changes
* Added service server to tesseract environment monitor for updating the environment
* Implemented move functionality for attached bodies
* Merge pull request `#69 <https://github.com/tesseract-robotics/tesseract_planning/issues/69>`_ from mpowelson/acm_test
  Add test to benchmark ACM isCollisionAllowed
* Add test to benchmark ACM isCollisionAllowed
* Merge pull request `#59 <https://github.com/tesseract-robotics/tesseract_planning/issues/59>`_ from arocchi/acm_fixes
  ACM  improvements: serialization and tests
* Fixed tesseract_ros::getActiveLinkNamesRecursive; added test
* Added missing main() in ros_tesseract_utils_unit.cpp
* Revert "fixed kdl_env_unit.cpp tests by using setActiveCollisionObjects() for continuous contact manager"
  This reverts commit c84bdc55d209a5a660c212c5d1a803f32d5390f6.
  This change should have been rendered obsolete by `#64 <https://github.com/tesseract-robotics/tesseract_planning/issues/64>`_
* Merge branch 'kinetic-devel' into acm_fixes
* Update contact managers active links when environment changes
* Add active link unit test
* fixed kdl_env_unit.cpp tests by using setActiveCollisionObjects() for continuous contact manager
* Fixed tests using discrete collision manager in kdl_env_unit.cpp
* Added installation of tesseract_monitoring launch files to CMakeLists
* Update to work with ReadTheDocs
* Add tesseract_collision documentation
* Fix clang formating
* Add clang-format AlignAfterOpenBracket: Align
* Export HACD library from bullet
* Fixed typo in documentation for AllowedCollisionEntry.msg
* Added AllowedCollisionEntry.msg in tesseract_msgs
* Fixes to run_clang_format_check
* Added unit tests for ros_tesseract_utils and KDLEnv
* Added test for AllowedCollisionMatrix in tesseract_core
* TesseractState includes information on allowed collisions, and ros_tesseract_utils are able to use them to correctly serialize and deserialize tesseract_ros::ROSBasicEnv from TesseractState messages
* Changed the AllowedCollisionMatrix lookup table to be indexed by std::pair<std::string, std::string>
* Fix formatting using clang
* Update travis ci to include clang-format test
* Fix warnings in unit tests
* Update due to changes in FCL Convex Shape Constructor
* Add additional compiler warning options
* Updated bullet_ros to not build unit tests; added line for installation of plugin XML files in Rviz package
* Implement topic subscriber for updating collision monitor environment
* Implement synchronous "compute_contact_reports" service in contact_monitor.cpp
* Eigen Alignment fixes
* Update due to change in trajopt
* Clean up TrajOptPlannerConfig doxygen
* Remove allocator for std::vector<Eigen::Vector3d>
* Fix axis plotting and add util for JointStateMsg
* Add monitoring of joint state topic to tesseract state display
* Simplify trajopt planner
* Ignore unused param warnings in bullet
* Add EIGEN_MAKE_ALIGNED_OPERATOR_NEW macros
* Fixed typo 'constacts' in ContactResultVector.msg
* Update bullet3 submodule to master
* Disable tesseract_collision FCL ConvexHull tests
* Fix/Clean depends in CMakeLists.txt and package.xml for travis-ci
* Create .travis.yml
* Fix trajopt planner collision check
* Merge pull request `#13 <https://github.com/tesseract-robotics/tesseract_planning/issues/13>`_ from mpowelson/trajopt_planning_fixes
  Got TrajoptPlanner in working order
* Trajopt Planner: Pass prob by ref
* Trajopt planning changes: Add docs and rearrange parameters
* Add callbacks/parameters to trajopt planner
* Minor Fix
* Clang Format
* Got TrajoptPlanner in working order
* Merge pull request `#41 <https://github.com/tesseract-robotics/tesseract_planning/issues/41>`_ from Levi-Armstrong/issue/FixMultiLayerCompoundShape
  Fix use of multi layer compound shape
  Fix/add cmake install commands
* Fix cmake install commands
* Fix use of multi layer compound shape
* Merge pull request `#40 <https://github.com/tesseract-robotics/tesseract_planning/issues/40>`_ from Levi-Armstrong/feature/RemoveContactRequestStruct
  Refractor out ContactRequest type
* Refractor out ContactRequest type
* Merge pull request `#38 <https://github.com/tesseract-robotics/tesseract_planning/issues/38>`_ from Jmeyer1292/melodic-fixes
  On Melodic, Fix Issue Where OMPL Package Is Not Found
* Merge pull request `#39 <https://github.com/tesseract-robotics/tesseract_planning/issues/39>`_ from Levi-Armstrong/issue/FixBulletCast
  This fixes the continuous collision checking
* Fix the use of ContactRequestType::FIRST with broadphase
* Fix cast bvh manager
* Fix bullet continous collision checking
* Find the lower-case ompl package. Fixes an issue where OMPL was not found my CMAKE. Note that Moveit made this transition in kinetic.
* Merge pull request `#34 <https://github.com/tesseract-robotics/tesseract_planning/issues/34>`_ from Levi-Armstrong/issue/FixBulletCast
  * This fixes the bullet cast simple manager
  * Fix the plotting of frames
  * Add unit test when using change base in kdl kin
  * Remove bullet build flags.
  * When adding use double precision this causes trajopt_ros test to fail. I believe this is due to inaccuracies in the EPA algorithm.
* Remove bullet build flags
  This for some reason causes TrajOpt to fail most likely due to bad results from the EPA algorithm
* Add unit test for using change base in kdl kin
* Fix the plotting of frames
* Merge pull request `#37 <https://github.com/tesseract-robotics/tesseract_planning/issues/37>`_ from arocchi/fix_36
  Changed destruction order of plugin loader and instantiation of loaded class
* Changed destruction order of plugin loader and instantiation of loaded class
* Fix compound and children aabb when updating cast transform
* Fix bullet cast simple manager
* Restructure bullet managers to be in separate files
* Merge pull request `#32 <https://github.com/tesseract-robotics/tesseract_planning/issues/32>`_ from Levi-Armstrong/issue/testCollisionClone
  Add unit test for clone method and fix mesh to mesh unit test names
* Fix kdl jacobian calculation and add unit tests
* Add unit test for clone method and fix mesh to mesh unit test names
* Merge pull request `#33 <https://github.com/tesseract-robotics/tesseract_planning/issues/33>`_ from Levi-Armstrong/issue/fixPluginDescription
  Fix namespace in plugin description
* Fix namespace in plugin description
* Merge pull request `#29 <https://github.com/tesseract-robotics/tesseract_planning/issues/29>`_ from Levi-Armstrong/issue/addCollisionNamespaces
  Add namespaces specific to collision implementation
* Fix lambda functions
* Add Bullet detailed mesh to detailed mesh collision checking along with unit test
* Adjust test to run for both primitive and convex shape.
* Add namespaces specific to collision implementation
* Merge pull request `#31 <https://github.com/tesseract-robotics/tesseract_planning/issues/31>`_ from Jmeyer1292/fixup/planning
  OMPL Planner Fixups
* Fixups required to use the OMPL planner effectively with other ROS systems. This includes a) a change to compute the transforms and set the transforms of the whole scene, and b) a fix to the CMakeLists to export the right libraries.
* Merge pull request `#26 <https://github.com/tesseract-robotics/tesseract_planning/issues/26>`_ from Levi-Armstrong/issue/FixContactMonitor
  Update contact monitor to use the latest version
* Merge pull request `#28 <https://github.com/tesseract-robotics/tesseract_planning/issues/28>`_ from Jmeyer1292/fix/bullet_include
  Bullet Convex Hull Computer Include
* Corrected include file path to work with the bullet3_ros package include paths
* Fix the contact monitor to use the new contact managers
* Fix asserts in CollisionObjectWrapper for bullet and fcl
* Merge pull request `#23 <https://github.com/tesseract-robotics/tesseract_planning/issues/23>`_ from Levi-Armstrong/feature/addFCLNew
  Add fcl discrete collision manager
* Make requested changes and fixes
* Add ros node for creating convex hull meshes
* Add fcl convex hull support and update tests
* Fix bullet cast assert in setCollisionObjectsTransform
* Add FCL discrete manager
* Fix comments from affine to isometry
* Merge pull request `#20 <https://github.com/tesseract-robotics/tesseract_planning/issues/20>`_ from Levi-Armstrong/feature/Isometry3d
  switch from using affine3d to isometry3d
* Add FCL and libccd submodules
* Add virtual destructors to base classes
* Add large octomap collision unit test enable aabb tree for compound shapes
* Update kdl chain kin unit to Isometry
* Convert to use templated using for vectors and maps of eigen types
* switch from using affine3d to isometry3d
* Merge pull request `#18 <https://github.com/tesseract-robotics/tesseract_planning/issues/18>`_ from Jmeyer1292/fix/add_manip
  addManipulator() checks for failure
* addManipulator() now ensures that the group could be created prior to returning success.
* Merge pull request `#17 <https://github.com/tesseract-robotics/tesseract_planning/issues/17>`_ from Levi-Armstrong/fixIssue7
  Fix issue `#7 <https://github.com/tesseract-robotics/tesseract_planning/issues/7>`_ with test
* Fix issue `#7 <https://github.com/tesseract-robotics/tesseract_planning/issues/7>`_ with test
* Merge pull request `#15 <https://github.com/tesseract-robotics/tesseract_planning/issues/15>`_ from Levi-Armstrong/feature/largeDataSetTest
  Restructure Collision Checking for Performance Improvements
* Update .clang-format comment
* Run clang-format
* Restructure Collision Checking for Performance Improvements
* Merge pull request `#16 <https://github.com/tesseract-robotics/tesseract_planning/issues/16>`_ from Levi-Armstrong/issue14
  Remove stray node handle
* Remove stray node handle
* Merge pull request `#5 <https://github.com/tesseract-robotics/tesseract_planning/issues/5>`_ from ros-industrial-consortium/Levi-Armstrong-patch-1
  Update README.md
* Update README.md
* Merge pull request `#3 <https://github.com/tesseract-robotics/tesseract_planning/issues/3>`_ from Levi-Armstrong/FixGHPages
  fix gh_pages config file
* fix gh_pages config file
* Merge pull request `#2 <https://github.com/tesseract-robotics/tesseract_planning/issues/2>`_ from Levi-Armstrong/addGHPages
  Add gh_pages for sphinx documentation
* Add gh_pages for sphinx documentation
* Merge pull request `#1 <https://github.com/tesseract-robotics/tesseract_planning/issues/1>`_ from Levi-Armstrong/fixSubmodule
  Fix submodule for bullet3
* Fix submodule for bullet3
* Move tesseract into its own repository
* Merge pull request `#12 <https://github.com/tesseract-robotics/tesseract_planning/issues/12>`_ from larmstrong/clangFormat
  clang format code, use Eigen::Ref and add kdl_joint_kin
* Add kdl_joint_kin to handle auxillary axes
* Fix kdl_chain_kin to handle links not in chain
* Add override to headers and clang format
* Make use of Eigen::Ref
* clang format code
* Merge pull request `#11 <https://github.com/tesseract-robotics/tesseract_planning/issues/11>`_ from larmstrong/unusedParamWarn
  Fix remaining warning
* Uncomment unused names in headers
* Add ability to check if two attachable objects and bodies are the same
* rename package tesseract_ros_planning to tesseract_planning
* Fix tesseract_monitoring package
* Add trajectory norm calculation to glass_up_right_plan.cpp
* Uncomment node tag in trajopt_examples launch files
* Fix planning_unit.cpp test
* Revert change in kdl_chain_kin.cpp
* Fix remaining warning
* Merge pull request `#10 <https://github.com/tesseract-robotics/tesseract_planning/issues/10>`_ from larmstrong/mergeJMeyer
  Merge jmeyer pull requests
* Merge pull request `#9 <https://github.com/tesseract-robotics/tesseract_planning/issues/9>`_ from larmstrong/removeOpenRave
  Merge removeOpenRave branch
* Fix failed rebase
* Remove append operation from AttachableObject
* Added continuous motion validator. We'll probably want a better API for this.
* Added some docs
* Implemented a basic interface to the OMPL SimpleSetup class for use with Tesseract
* Remove append operation from AttachableObject
* Removed warnings again. Just too many in included libraries to deal with.
* Gobs more small fixups. I don't believe I changed anything that would affect actual logic.
* Removed use of deprecated JSON_CPP function calls
* Cleaning up warnings
* Set the transform for the attached body in the glass upright example
* Fix contact monitoring
* Fix contact plugin loader crashing
* Merge branch 'removeOpenRave' of https://raesgit.datasys.swri.edu/larmstrong/trajopt_ros into removeOpenRave
* Intialize AttachedBodyInfo transform to identity
* Rename isCollisionAllowed to isContactAllowed
* Merge pull request `#4 <https://github.com/tesseract-robotics/tesseract_planning/issues/4>`_ from Jmeyer/fix/no_include
  Trajopt Tools Include Directory
* Removed include directory references to an include directory that doesn't exist
* Create custom rviz environment plugin
* Add Car Seat Example
* Add a few bullet3 build options
* Fix glass_up_right_plan, need to add collision obj type
* Clean up trajopt_ext CMakeLists.txt file
* Add branch to submodule
* Add submodule to bullet3
* Remove copy of bullet
* Add ability to define collision object type
* Remove SHAPE_EXPANSION from bullet
* Disable moveit related packages
* Refractor collision checking into its own package
* Add tesseract_ros_monitor with environment monitor
* Switch boost::function to std::function
* Switch boost::shared_ptr to std::shared_ptr
* Fix getActiveLinkNamesRecursive method
* Add method getActiveLinkNames to env base class
* Add parameters to contact_monitoring node
* Add package tesseract_ros_planning package
* Add basic environment singleton and contact monitoring node
* Add missing license information
* Add getJointValues() and getLinkTransforms() to basic_env.h
* Rename DistanceRequest DistanceResults to ContactRequest ContactResults
* Separate Plotting from environment and fix object color typedef
* In examples add node to launch file
* Add tesseract packages
* replace std::map with std::unordered_map
* Make AllowedCollisionMatrix a class
* replace trajopt_scene with tesseract package
* Add ability to set safety margin for link pairs
* Move data directory content to trajopt_test_support/config directory
* Remove const from std::map key
* Add ability to visualize trajopt_scene using robot state
* Move moveit items to its own package and create trajopt_scene package
* Remove moveit depend from ros_kin_chain
* Add system depend to CMakeLists.txt
* Fix bug in collision_common.h
* Add ability to get global minimum for pair instead of just all
* Tune optimization parameters for puzzle piece example
* Move the plotWaitForInput to the plot callback function
* Rename ROSKin to ROSKinChain and add JointAccCost JointJerkCost
* Rename getManipulatorKin to getManipulator
* Add alternative continuousCollisionCheckTrajectory function
* enable collision for puzzle piece example
* Integrate changes to moveit collision
* Add trajopt_tools package with hacd and vhacd
* Add vhacd to trajopt_ext
* Add puzzle piece example
* Add tcp capability to kinematics_terms
* Remove debug code from glass_up_right_plan.cpp
* Update the iiwa dae to be shadeless
* Switch examples to use kuka iiwa copied into this pacakge
* Fix commented out plotting calls
* Add ability to publish axes
* Remove additional refferences to openrave
* Make distance and collision calls const and fix ROS_INFO warnings
* Add glass up right example
* Expose optimization parameters to user via cpp and json
* Remove the use of global ProblemConstructionInfo variable when parsing json data
* Add trajopt_examples package with one cartesian example
* Remove old json unit tests
* Remove old test collision-checker-unit
* Restructure trajopt_ext directory
* Remove local version of jsoncpp
* Remove pr2 moveit_cofig package
* Add octomap unit test and fix convert bullet convertBulletCollisions
* Add test for objects attached to links without geometry
* Fix bullet collision to handle attached object connected to links without geometry
* Fix use of attached collision objects and add a unit test for it
* Make use of BULLET_DEFAULT_CONTACT_DISTANCE
* Implement remaining collision_robot bullet methods
* Add attached object functionality
* Add collision world test and make use of xacros
* Integrate collision world
* Update isCollision allowed to handle Attached objects
* Change link2cow typedef
* Remove temp file
* Add/Update cast cost unit test
* Remove openrave utils
* Remove unused file pr2.gv and pr2.pdf
* Remove osgviewer package
* Switch planning unit test to use ROS_DEBUG
* Fix continuous collision checking and add original cast method
* Add Continuous Collision Checking and Filter Masking
* Add plotting parameter to trajopt_planning_unit
* MoveIt Bullet Collision Checker (Single State)
* Second pass at planning-unit test
* First pass at planning-unit test
* Working numerical ik test
* Fixup
* Add test support package and moveit config package
* Divide package into multiple packages
* Clean up CMakeLists.txt file
* Get tests working
* Minor fixes for finding openrave and openscenegraph
* TrajOpt Successful Build
* initial commit with sco utils and json building
* Initial commit
* Contributors: Alessio Rocchi, Armstrong, Levi H, Colin Lewis, David Merz, Jr, DavidMerzJr, Hervé Audren, John Wason, Jonathan Meyer, Joseph Schornak, Josh Langsfeld, Levi, Levi Armstrong, Levi-Armstrong, Marco Bassa, Matthew Powelson, Michael Ripperger, Patrick Beeson, Reid Christopher, Tyler Marr, dmerz, jrgnicho, marrts, mpowelson, mripperger
