^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_examples
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Move TaskComposerProblem input to base class and change type to tesseract_common::AnyPoly
* Unused includes cleanup
* Update based on changes in trajopt
* Contributors: Levi Armstrong, Roelof Oomen

0.20.1 (2023-10-02)
-------------------

0.20.0 (2023-09-29)
-------------------
* Update tesseract examples
* Rename TaskComposerInput to TaskComposerContext and simplify interfaces (`#379 <https://github.com/tesseract-robotics/tesseract_planning/issues/379>`_)
* Contributors: Levi Armstrong

0.19.0 (2023-09-05)
-------------------
* Update to leverage cmake components
* Disable GlassUprightTrajOptIfoptExampleUnit Test
* Contributors: Levi Armstrong

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
* Upgrade to TrajOpt 0.6.0
* Fixes for Python wrappers (`#329 <https://github.com/tesseract-robotics/tesseract_planning/issues/329>`_)
* Add TrajOpt multi threaded support
* Restructure tesseract_task_composer like other plugin based packages
* Add PlanningTaskComposerProblem
* Contributors: John Wason, Levi Armstrong

0.17.0 (2023-06-06)
-------------------

0.16.3 (2023-05-03)
-------------------

0.16.2 (2023-04-28)
-------------------

0.16.1 (2023-04-11)
-------------------

0.16.0 (2023-04-09)
-------------------
* Fix clang-tidy errors
* Fixing OMPL freespace example and a typo (`#299 <https://github.com/tesseract-robotics/tesseract_planning/issues/299>`_)
  * Fix freespace OMPL example (was hybrid)
  * Fix typo in iterative_spline_parameterization_profile file name
* Contributors: Levi Armstrong, Roelof

0.15.5 (2023-03-22)
-------------------

0.15.4 (2023-03-16)
-------------------
* Update example to include geometry headers
* Contributors: Levi Armstrong

0.15.3 (2023-03-15)
-------------------

0.15.2 (2023-03-14)
-------------------

0.15.1 (2023-03-09)
-------------------

0.15.0 (2023-03-03)
-------------------
* Update task composer to leverage plugins (`#282 <https://github.com/tesseract-robotics/tesseract_planning/issues/282>`_)
* Remove composite start instruction
* Contributors: Levi Armstrong

0.14.0 (2022-10-23)
-------------------
* Update to use ModifyAllowedCollisionCommand
* Fix trajopt ifopt planner
* Add convex solver config to TrajOpt solver profile and fix puzzle piece aux example
* Remove references to tesseract process managers package
* Add tesseract_task_composer package to replace tesseract_process_managers
* Remove use of tesseract_common::StatusCode
* Contributors: Levi Armstrong

0.13.1 (2022-08-30)
-------------------

0.13.0 (2022-08-25)
-------------------
* Rename tesseract_command_language core directory to poly
* Rename Waypoint and Instruction to WaypointPoly and InstructionPoly
* Add CartesianWaypointPoly, JointWaypointPoly and StateWaypointPoly
* Refactor using MoveInstructionPoly
* Remove plan instruction
* Contributors: Levi Armstrong

0.12.0 (2022-07-07)
-------------------

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
* Update ProcessPlanningFuture to leverage shared future (`#188 <https://github.com/tesseract-robotics/tesseract_planning/issues/188>`_)
  * Update ProcessPlanningFuture to leverage shared future
  * fix problem swid def
* Contributors: Levi Armstrong

0.9.8 (2022-04-19)
------------------

0.9.7 (2022-04-08)
------------------

0.9.6 (2022-04-01)
------------------
* Add tesseract_examples package
* Contributors: Levi Armstrong
