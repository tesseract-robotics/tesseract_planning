^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_task_composer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
