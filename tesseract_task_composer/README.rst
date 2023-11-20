=======================
Tesseract Task Composer
=======================

A interface for general purpose parallel task programming system. 

- TaskComposerExecutor
  - A interface for executing a task or graph of task
- TaskComposerNode
  - A interface for implementing an task or graph
- TaskComposerServer
  - Loads a yaml config which import TaskComposerExecutor and TaskComposerNode plugins.

Task Composer Plugin Config
---------------------------

This file allows you define Excutors and Tasks (aka Nodes). 

.. note:: 
    
   Not all nodes are intendend to be standalone task but be comsumed by a task.

.. code-block:: yaml

   task_composer_plugins:
     search_paths:
       - /usr/local/lib
     search_libraries:
       - tesseract_task_composer_factories
     executors:
       default: TaskflowExecutor
       plugins:
         TaskflowExecutor:
           class: TaskflowTaskComposerExecutorFactory
           config:
             threads: 5
     tasks:
       plugins:
         DescartesFPipeline:
           class: GraphTaskFactory
           config:
             inputs: [input_data]
             outputs: [output_data]
             nodes:
               DoneTask:
                 class: DoneTaskFactory
                 config:
                   conditional: false
               ErrorTask:
                 class: ErrorTaskFactory
                 config:
                   conditional: false
               MinLengthTask:
                 class: MinLengthTaskFactory
                 config:
                   conditional: true
                   inputs: [input_data]
                   outputs: [output_data]
               DescartesMotionPlannerTask:
                 class: DescartesFMotionPlannerTaskFactory
                 config:
                   conditional: true
                   inputs: [output_data]
                   outputs: [output_data]
                   format_result_as_input: false
               DiscreteContactCheckTask:
                 class: DiscreteContactCheckTaskFactory
                 config:
                   conditional: true
                   inputs: [output_data]
               IterativeSplineParameterizationTask:
                 class: IterativeSplineParameterizationTaskFactory
                 config:
                   conditional: true
                   inputs: [output_data]
                   outputs: [output_data]
             edges:
               - source: MinLengthTask
                 destinations: [DescartesMotionPlannerTask]
               - source: DescartesMotionPlannerTask
                 destinations: [ErrorTask, DiscreteContactCheckTask]
               - source: DiscreteContactCheckTask
                 destinations: [ErrorTask, IterativeSplineParameterizationTask]
               - source: IterativeSplineParameterizationTask
                 destinations: [ErrorTask, DoneTask]
             terminals: [ErrorTask, DoneTask]
   
Task Composer Executors Plugins
-------------------------------

Task composer executors are populated under executors section in the config above.

.. note:: 
   The name given to the executor can be anything and this is the executor name that should be used when requests are made in TaskComposerServer or TaskComposerPluginFactory.

Taskflow
^^^^^^^^
Yaml Config:

.. code-block:: yaml

   TaskflowExecutor:
     class: TaskflowTaskComposerExecutorFactory
     config:
       threads: 5


Task Composer Task Plugins
--------------------------

Task
^^^^

All tasks have the following config entries available.

.. code-block:: yaml

   ErrorTask:
     class: ErrorTaskFactory
     config:
       conditional: false
       trigger_abort: true # default for task is false
       inputs: [input_data]
       outputs: [output_data]


Graph Task
^^^^^^^^^^

Task for composing graph of tasks. A node in the graph can be a plugin or previously defined task.

Define the graph nodes and edges as shown in the config below.

.. code-block:: yaml

   CartesianPipeline:
     class: PipelineTaskFactory
     config:
       inputs: [input_data]
       outputs: [output_data]
       nodes:
         DoneTask:
           class: DoneTaskFactory
           config:
             conditional: false
         ErrorTask:
           class: ErrorTaskFactory
           config:
             conditional: false
         MinLengthTask:
           class: MinLengthTaskFactory
           config:
             conditional: true
             inputs: [input_data]
             outputs: [output_data]
         DescartesMotionPlannerTask:
           class: DescartesFMotionPlannerTaskFactory
           config:
             conditional: true
             inputs: [output_data]
             outputs: [output_data]
             format_result_as_input: true
         TrajOptMotionPlannerTask:
           class: TrajOptMotionPlannerTaskFactory
           config:
             conditional: true
             inputs: [output_data]
             outputs: [output_data]
             format_result_as_input: false
         DiscreteContactCheckTask:
           class: DiscreteContactCheckTaskFactory
           config:
             conditional: true
             inputs: [output_data]
         IterativeSplineParameterizationTask:
           class: IterativeSplineParameterizationTaskFactory
           config:
             conditional: true
             inputs: [output_data]
             outputs: [output_data]
       edges:
         - source: MinLengthTask
           destinations: [DescartesMotionPlannerTask]
         - source: DescartesMotionPlannerTask
           destinations: [ErrorTask, TrajOptMotionPlannerTask]
         - source: TrajOptMotionPlannerTask
           destinations: [ErrorTask, DiscreteContactCheckTask]
         - source: DiscreteContactCheckTask
           destinations: [ErrorTask, IterativeSplineParameterizationTask]
         - source: IterativeSplineParameterizationTask
           destinations: [ErrorTask, DoneTask]
       terminals: [ErrorTask, DoneTask]

Leveraging a perviously defined task
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

When using a perviously defined task it is referenced using `task:` instead of `class:`. 

Also in most case the tasks inputs and sometimes the outputs must be renamed. This accomplished by leveraging the `remapping:`.

Also you can indicate that it should abort if a terminal is reached by specifying the terminal index `abort_terminal:`. If set to anything less than zero it will set all terminal tasks trigger abort to `false`.

.. code-block:: yaml

   UsePreviouslyDefinedTaskPipeline:
     class: PipelineTaskFactory
     config:
       inputs: [input_data]
       outputs: [output_data]
       nodes:
         MinLengthTask:
           class: MinLengthTaskFactory
           config:
             conditional: true
             inputs: [input_data]
             outputs: [output_data]
         CartesianPipelineTask:
            task: CartesianPipeline
            config:
              conditional: false         # Optional
              abort_terminal: 0          # Optional
              remapping:           # Optional
                input_data: output_data
       edges:
         - source: MinLengthTask
           destinations: [CartesianPipelineTask]
       terminals: [CartesianPipelineTask]

Descartes Motion Planner Task
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Task for running Descartes motion planner

.. note:: This is using double.

.. code-block:: yaml

   DescartesMotionPlannerTask:
     class: DescartesDMotionPlannerTaskFactory
     config:
       conditional: true
       inputs: [input_data]
       outputs: [output_data]
       format_result_as_input: false


.. note:: This is using float

.. code-block:: yaml

   DescartesMotionPlannerTask:
     class: DescartesFMotionPlannerTaskFactory
     config:
       conditional: true
       inputs: [input_data]
       outputs: [output_data]
       format_result_as_input: false

OMPL Motion Planner Task
^^^^^^^^^^^^^^^^^^^^^^^^

Task for running OMPL motion planner

.. code-block:: yaml

   OMPLMotionPlannerTask:
     class: OMPLMotionPlannerTaskFactory
     config:
       conditional: true
       inputs: [input_data]
       outputs: [output_data]
       format_result_as_input: false

TrajOpt Motion Planner Task
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Task for running TrajOpt motion planner

.. code-block:: yaml

   TrajOptMotionPlannerTask:
     class: TrajOptMotionPlannerTaskFactory
     config:
       conditional: true
       inputs: [input_data]
       outputs: [output_data]
       format_result_as_input: false

TrajOpt Ifopt Motion Planner Task
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Task for running TrajOpt Ifopt motion planner

.. code-block:: yaml

   TrajOptIfoptMotionPlannerTask:
     class: TrajOptIfoptMotionPlannerTaskFactory
     config:
       conditional: true
       inputs: [input_data]
       outputs: [output_data]
       format_result_as_input: false

Simple Motion Planner Task
^^^^^^^^^^^^^^^^^^^^^^^^^^

Task for running Simple motion planner

.. code-block:: yaml

   SimpleMotionPlannerTask:
     class: SimpleMotionPlannerTaskFactory
     config:
       conditional: true
       inputs: [input_data]
       outputs: [output_data]
       format_result_as_input: true

Iterative Spline Parameterization Task
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Perform iterative spline time parameterization

.. code-block:: yaml

   IterativeSplineParameterizationTask:
     class: IterativeSplineParameterizationTaskFactory
     config:
       conditional: true
       inputs: [input_data]
       outputs: [output_data]
       add_points: true # optional

Time Optimal Time Parameterization Task
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Perform time optimal time parameterization

.. code-block:: yaml

   TimeOptimalParameterizationTask:
     class: TimeOptimalParameterizationTaskFactory
     config:
       conditional: true
       inputs: [input_data]
       outputs: [output_data]

Ruckig Trajectory Smoothing Task
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Perform trajectory smoothing leveraging Ruckig. Time parameterization must be ran before using this task.

.. code-block:: yaml

   RuckigTrajectorySmoothingTask:
     class: RuckigTrajectorySmoothingTaskFactory
     config:
       conditional: true
       inputs: [input_data]
       outputs: [output_data]

Raster Motion Task
^^^^^^^^^^^^^^^^^^

.. code-block:: yaml

   RasterMotionTask:
     class: RasterMotionTaskFactory
     config:
       conditional: true
       inputs: [output_data]
       outputs: [output_data]
       freespace:
         task: FreespacePipeline
         config:
           remapping:
             input_data: output_data
           indexing: [output_data]
       raster:
         task: CartesianPipeline
         config:
           remapping:
             input_data: output_data
           indexing: [output_data]
       transition:
         task: FreespacePipeline
         config:
           remapping:
             input_data: output_data
           indexing: [output_data]

Raster Only Motion Task
^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: yaml

   RasterMotionTask:
     class: RasterOnlyMotionTaskFactory
     config:
       conditional: true
       inputs: [output_data]
       outputs: [output_data]
       raster:
         task: CartesianPipeline
         config:
           remapping:
             input_data: output_data
           indexing: [output_data]
       transition:
         task: FreespacePipeline
         config:
           remapping:
             input_data: output_data
           indexing: [output_data]


Check Input Task
^^^^^^^^^^^^^^^^

Task for checking input data structure

.. code-block:: yaml

   MinLengthTask:
     class: MinLengthTaskFactory
     config:
       conditional: true
       inputs: [input_data]
       outputs: [output_data]

Continuous Contact Check Task
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Continuous collision check trajectory task

.. code-block:: yaml

   ContinuousContactCheckTask:
     class: ContinuousContactCheckTaskFactory
     config:
       conditional: true
       inputs: [input_data]

Discrete Contact Check Task
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Discrete collision check trajectory task

.. code-block:: yaml

   DiscreteContactCheckTask:
     class: DiscreteContactCheckTaskFactory
     config:
       conditional: true
       inputs: [input_data]

Done Task
^^^^^^^^^

The final task that is called in a task graph if successful

.. code-block:: yaml

   DoneTask:
     class: DoneTaskFactory
     config:
       conditional: false

Error Task
^^^^^^^^^^

The final task that is called in a task graph if error occurs. Also can add `trigger_abort: true` if it should abort when this task is reached.

.. code-block:: yaml

   ErrorTask:
     class: ErrorTaskFactory
     config:
       conditional: false

Sync Task
^^^^^^^^^

The task is used to create a syncronization point within a task graph

.. code-block:: yaml

   SyncTask:
     class: SyncTaskFactory
     config:
       conditional: false

Remap Task
^^^^^^^^^^

Remap data from one key to another, by copying or moving the data.

.. code-block:: yaml

   RemapTask:
     class: RemapTaskFactory
     config:
       conditional: false
       copy: true
       remap:
         key1: remap_key1
         key2: remap_key2

Fix State Bounds Task
^^^^^^^^^^^^^^^^^^^^^

This task modifies the input instructions in order to push waypoints that are outside of their limits back within them.

.. code-block:: yaml

   FixStateBoundsTask:
     class: FixStateBoundsTaskFactory
     config:
       conditional: true
       inputs: [input_data]
       outputs: [output_data]

Fix State Collision Task
^^^^^^^^^^^^^^^^^^^^^^^^

This task modifies the input instructions in order to push waypoints that are in collision out of collision.

.. note:: 
   First it uses TrajOpt to correct the waypoint. If that fails, it reverts to random sampling

.. code-block:: yaml

   FixStateCollisionTask:
     class: FixStateCollisionTaskFactory
     config:
       conditional: true
       inputs: [input_data]
       outputs: [output_data]

Min Length Task
^^^^^^^^^^^^^^^

Task for processing the input data so it meets a minimum length. Planners like trajopt need at least 10 states in the trajectory to perform velocity, acceleration and jerk smoothing.

.. code-block:: yaml

   MinLengthTask:
     class: MinLengthTaskFactory
     config:
       conditional: false
       inputs: [input_data]
       outputs: [output_data]

Profile Switch Task
^^^^^^^^^^^^^^^^^^^

This task simply returns a value specified in the composite profile. This can be used to switch execution based on the profile

.. code-block:: yaml

   ProfileSwitchTask:
     class: ProfileSwitchTaskFactory
     config:
       conditional: false
       inputs: [input_data]

Upsample Trajectory Task
^^^^^^^^^^^^^^^^^^^^^^^^

This is used to upsample the results trajectory based on the longest valid segment length.

.. note:: 
   This is primarily useful to run before running time parameterization, because motion planners assume joint interpolated between states. If the points are spaced to fart apart the path between two states may not be a straight line causing collision during execution.

.. code-block:: yaml

   UpsampleTrajectoryTask:
     class: UpsampleTrajectoryTaskFactory
     config:
       conditional: false
       inputs: [input_data]
       outputs: [output_data]

Format As Input Task
^^^^^^^^^^^^^^^^^^^^

This is used in the case where you run trajopt with collision as a cost and then you post check it for collision and it fails. Then you run trajopt with collision as a constraint but the output from trajopt with collision as a cost must be formated as input for trajopt with collision as a constraint planner.

This will take the results stored in input_keys[1] and store it in the input_keys[0] program and save the results in the output key.

 - input_keys[0]: The original input to motion planning
 - input_keys[1]: The output of the first motion plan which failed collision checking

.. code-block:: yaml

   FormatAsInputTask:
     class: FormatAsInputTaskFactory
     config:
       conditional: false
       inputs: [input_pre_data, input_post_data]
       outputs: [output_data]
