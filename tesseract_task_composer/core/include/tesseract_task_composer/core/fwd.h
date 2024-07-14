#ifndef TESSERACT_TASK_COMPOSER_CORE_FWD_H
#define TESSERACT_TASK_COMPOSER_CORE_FWD_H

namespace tesseract_planning
{
// task_composer_context.h
class TaskComposerContext;

// task_composer_data_storage.h
class TaskComposerDataStorage;

// task_composer_executor.h
class TaskComposerExecutor;

// task_composer_future.h
class TaskComposerFuture;

// task_composer_graph.h
class TaskComposerGraph;

// task_composer_keys.h
class TaskComposerKeys;

// task_composer_node_info.h
class TaskComposerNodeInfo;
class TaskComposerNodeInfoContainer;

// task_composer_node.h
enum class TaskComposerNodeType;
class TaskComposerNode;
class TaskComposerNodePorts;

// task_composer_pipeline.h
class TaskComposerPipeline;

// task_composer_plugin_factory.h
class TaskComposerNodeFactory;
class TaskComposerExecutorFactory;
class TaskComposerPluginFactory;

// task_composer_server.h
class TaskComposerServer;

// task_composer_task.h
class TaskComposerTask;

// nodes
class DoneTask;
class ErrorTask;
class RemapTask;
class StartTask;
class SyncTask;

}  // namespace tesseract_planning

#endif  // TESSERACT_TASK_COMPOSER_CORE_FWD_H
