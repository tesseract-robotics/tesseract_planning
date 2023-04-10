^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_task_composer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
