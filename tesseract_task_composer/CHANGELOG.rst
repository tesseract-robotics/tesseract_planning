^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_task_composer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.21.5 (2023-12-13)
-------------------
* Fix TaskComposerProblem serialization and equal operator
* Contributors: Levi Armstrong

0.21.4 (2023-11-21)
-------------------

0.21.3 (2023-11-20)
-------------------
* Update README.rst
  Description of Simple Motion Planner task fixed
* Contributors: Roelof

0.21.2 (2023-11-17)
-------------------
* Improve dynamic tasking support
* Contributors: Levi Armstrong

0.21.1 (2023-11-17)
-------------------
* Fix loss of first waypoint in upsample trajectory (`#416 <https://github.com/tesseract-robotics/tesseract_planning/issues/416>`_)
* Use taskflow subflow for graph execution to allow timing of execution
* Contributors: Levi Armstrong, Thomas Hettasch

0.21.0 (2023-11-10)
-------------------
* Fix clang-tidy errors
* Replace input_indexing and output_indexing with indexing
* Replace input_remapping and output_remapping with remapping
* Move TaskComposerProblem input to base class and change type to tesseract_common::AnyPoly
* remove results from TaskComposerNodeInfo
* Unused includes cleanup
* Contributors: Levi Armstrong, Roelof Oomen

0.20.1 (2023-10-02)
-------------------

0.20.0 (2023-09-29)
-------------------
* Remove AbortTask
* Add input instruction to planning problem
* Merge pull request `#370 <https://github.com/tesseract-robotics/tesseract_planning/issues/370>`_ from marip8/update/task-composer-factory-constructor
  Add new task composer plugin factory constructor
* Added unit test for new TaskComposerPluginFactory constructor
* Added constructor to task composer plugin factory to use task composer plugin config struct
* Rename TaskComposerInput to TaskComposerContext and simplify interfaces (`#379 <https://github.com/tesseract-robotics/tesseract_planning/issues/379>`_)
* Contributors: Levi Armstrong, Michael Ripperger

0.19.0 (2023-09-05)
-------------------
* Update to leverage cmake components
* Fix Raster and RasterOnly Tasks
* Add elapsed time for pipelines and include in dot graph
* Add conditional to subgraph in dot graph output
* Add input and output keys to dot graph
* Add Remap Task (`#351 <https://github.com/tesseract-robotics/tesseract_planning/issues/351>`_)
* Contributors: Levi Armstrong

0.18.4 (2023-07-07)
-------------------
* Move task composer elapse timing to base classes
* Contributors: Levi Armstrong

0.18.3 (2023-07-04)
-------------------
* Fix MotionPlannerTaskInfo serialization
* Contributors: Levi Armstrong

0.18.2 (2023-07-03)
-------------------
* Add clone method to TaskComposerProblem
* Contributors: Levi Armstrong

0.18.1 (2023-07-03)
-------------------
* Fix TaskComposerServer destruction
* Contributors: Levi Armstrong

0.18.0 (2023-06-30)
-------------------
* Update task_composer_plugins_no_trajopt_ifopt.yaml
* Restruct Raster yaml config to have same look as everything else
* Leverage AbortTask and make ErrorTask not abort
* Remove unused file
* Upgrade to TrajOpt 0.6.0
* Add task composer planning unit tests (`#341 <https://github.com/tesseract-robotics/tesseract_planning/issues/341>`_)
* Fixes for Python wrappers (`#329 <https://github.com/tesseract-robotics/tesseract_planning/issues/329>`_)
* Add TaskComposerServer unit tests
* Add task composer taskflow unit tests (`#339 <https://github.com/tesseract-robotics/tesseract_planning/issues/339>`_)
* Add TaskComposerPipeline and improve task composer code coverage (`#337 <https://github.com/tesseract-robotics/tesseract_planning/issues/337>`_)
* Added trajectory logger printout to trajectory checker (`#338 <https://github.com/tesseract-robotics/tesseract_planning/issues/338>`_)
* Added an extra needed #include for 22.04 builds (`#332 <https://github.com/tesseract-robotics/tesseract_planning/issues/332>`_)
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Restructure tesseract_task_composer like other plugin based packages
* Add PlanningTaskComposerProblem
* Added ability to colorize dotgraphs with planning results (`#327 <https://github.com/tesseract-robotics/tesseract_planning/issues/327>`_)
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Contributors: John Wason, Levi Armstrong, Tyler Marr

0.17.0 (2023-06-06)
-------------------
* Fix Key Naming Scheme in Raster Motion Task  (`#324 <https://github.com/tesseract-robotics/tesseract_planning/issues/324>`_)
  @marrts Great find and thanks for the fix.
* Fix task composer cmake plugins variable
* Update task nodes to on failure store input in output location to better support error branching
* Fix some typos
* Contributors: Levi Armstrong, Roelof Oomen, Tyler Marr

0.16.3 (2023-05-03)
-------------------
* Fix FormatAsInputTask to store results
* Contributors: Levi Armstrong

0.16.2 (2023-04-28)
-------------------

0.16.1 (2023-04-11)
-------------------

0.16.0 (2023-04-09)
-------------------
* Add FormatAsInputTask
* Update to leverage CollisionCheckProgramType in collision config
* Fix clang-tidy errors
* Update to support new contact results class (`#297 <https://github.com/tesseract-robotics/tesseract_planning/issues/297>`_)
* Fixing OMPL freespace example and a typo (`#299 <https://github.com/tesseract-robotics/tesseract_planning/issues/299>`_)
  * Fix freespace OMPL example (was hybrid)
  * Fix typo in iterative_spline_parameterization_profile file name
* Add TOTG Node Info class
* Contributors: Levi Armstrong, Roelof

0.15.5 (2023-03-22)
-------------------
* Fix TOTG assignData
* Add fix_state_collision clone method and serialize contact results
* Build fixes for Focal/Foxy and Jammy/Humble
* Contributors: Levi Armstrong, Roelof Oomen

0.15.4 (2023-03-16)
-------------------

0.15.3 (2023-03-15)
-------------------

0.15.2 (2023-03-14)
-------------------
* Clean up task composer serialization
* Contributors: Levi Armstrong

0.15.1 (2023-03-09)
-------------------
* Add method for retrieving task from TaskComposerServer
* Use try catch in TaskComposerTask run because exceptions are not propagated in multi threaded runs.
* Update fix state bounds task to ignore cartesian waypoint types
* Contributors: Levi Armstrong

0.15.0 (2023-03-03)
-------------------
* Update task composer to leverage plugins (`#282 <https://github.com/tesseract-robotics/tesseract_planning/issues/282>`_)
* Use templates for raster task to reduce code duplications (`#279 <https://github.com/tesseract-robotics/tesseract_planning/issues/279>`_)
* Add descartes no post check motion pipeline task
* clean up update end state task
* Fix descartes global motion pipeline task
* Merge pull request `#269 <https://github.com/tesseract-robotics/tesseract_planning/issues/269>`_ from marip8/update/time-param-org
  Added optional builds of time parameterization implementations
* Created separate targets for each time parameterization implementation
* Updated task composer package
* Remove composite start instruction
* Add uuid and parent_uuid to InstructionPoly (`#261 <https://github.com/tesseract-robotics/tesseract_planning/issues/261>`_)
* Contributors: Levi Armstrong, Michael Ripperger

0.14.0 (2022-10-23)
-------------------
* Add ompl to default tasks utility function
* Fix trajopt ifopt task name
* Add environment to TaskComposerNodeInfo
* Add method to TaskComposerDataStorage to get copy of all data
* Update TaskComposerNodeInfo contructor to take node type
* Remove tesseract_process_managers package
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
* Add transition mux task
* Add equal operators to task composer tasks
* Remove use of tesseract_common::StatusCode
* Add task composer package
* Contributors: Levi Armstrong, Roelof Oomen
