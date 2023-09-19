
#include <iostream>
#include <tesseract_common/utils.h>
#include <tesseract_task_composer/core/task_composer_graph.h>
#include <tesseract_task_composer/core/task_composer_task.h>
#include <tesseract_task_composer/core/task_composer_problem.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>
#include <tesseract_task_composer/taskflow/taskflow_task_composer_executor.h>

using namespace tesseract_planning;

class AddTaskComposerNode : public TaskComposerTask
{
public:
  AddTaskComposerNode(std::string left_key, std::string right_key, std::string output_key)
    : TaskComposerTask("AddTwoNumbers", false)
    , left_key_(std::move(left_key))
    , right_key_(std::move(right_key))
    , output_key_(std::move(output_key))
  {
  }

  TaskComposerNodeInfo::UPtr runImpl(TaskComposerContext& context,
                                     OptionalTaskComposerExecutor /*executor*/) const override final
  {
    auto info = std::make_unique<TaskComposerNodeInfo>(*this);
    info->return_value = 0;
    std::cout << name_ << std::endl;
    double result =
        context.data_storage->getData(left_key_).as<double>() + context.data_storage->getData(right_key_).as<double>();
    context.data_storage->setData(output_key_, result);
    return info;
  }

protected:
  std::string left_key_;
  std::string right_key_;
  std::string output_key_;
};

class MultiplyTaskComposerNode : public TaskComposerTask
{
public:
  MultiplyTaskComposerNode(std::string left_key, std::string right_key, std::string output_key)
    : TaskComposerTask("MultiplyTwoNumbers", false)
    , left_key_(std::move(left_key))
    , right_key_(std::move(right_key))
    , output_key_(std::move(output_key))
  {
  }

  TaskComposerNodeInfo::UPtr runImpl(TaskComposerContext& context,
                                     OptionalTaskComposerExecutor /*executor*/) const override final
  {
    auto info = std::make_unique<TaskComposerNodeInfo>(*this);
    info->return_value = 0;
    std::cout << name_ << std::endl;
    double result =
        context.data_storage->getData(left_key_).as<double>() * context.data_storage->getData(right_key_).as<double>();
    context.data_storage->setData(output_key_, result);
    return info;
  }

protected:
  std::string left_key_;
  std::string right_key_;
  std::string output_key_;
};

int main()
{
  double a{ 1 };
  double b{ 3 };
  double c{ 5 };
  double d{ 9 };
  auto task_data = std::make_unique<TaskComposerDataStorage>();
  task_data->setData("a", a);
  task_data->setData("b", b);
  task_data->setData("c", c);
  task_data->setData("d", d);

  auto task_problem = std::make_unique<TaskComposerProblem>();

  // result = a * (b + c) + d
  auto task1 = std::make_unique<AddTaskComposerNode>("b", "c", "task1_output");
  auto task2 = std::make_unique<MultiplyTaskComposerNode>("a", "task1_output", "task2_output");
  auto task3 = std::make_unique<AddTaskComposerNode>("task2_output", "d", "task3_output");

  TaskComposerGraph task_composer;
  boost::uuids::uuid task1_id = task_composer.addNode(std::move(task1));
  boost::uuids::uuid task2_id = task_composer.addNode(std::move(task2));
  boost::uuids::uuid task3_id = task_composer.addNode(std::move(task3));
  task_composer.addEdges(task1_id, { task2_id });
  task_composer.addEdges(task2_id, { task3_id });

  const std::string share_dir(TESSERACT_TASK_COMPOSER_DIR);
  tesseract_common::fs::path config_path(share_dir + "/config/task_composer_plugins.yaml");
  TaskComposerPluginFactory factory(config_path);

  auto task_executor = factory.createTaskComposerExecutor("TaskflowExecutor");
  TaskComposerFuture::UPtr future = task_executor->run(task_composer, std::move(task_problem), std::move(task_data));
  future->wait();

  std::cout << "Output: " << future->context->data_storage->getData("task3_output").as<double>() << std::endl;
}
