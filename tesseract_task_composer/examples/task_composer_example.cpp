
#include <iostream>
#include <tesseract_common/utils.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_future.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_graph.h>
#include <tesseract_task_composer/core/task_composer_executor.h>
#include <tesseract_task_composer/core/task_composer_task.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>
#include <tesseract_task_composer/taskflow/taskflow_task_composer_executor.h>

using namespace tesseract::task_composer;

class AddTaskComposerNode : public TaskComposerTask
{
public:
  // Requried
  static const std::string INPUT_LEFT_PORT;
  static const std::string INPUT_RIGHT_PORT;
  static const std::string OUTPUT_RESULT_PORT;

  AddTaskComposerNode(std::string left_key, std::string right_key, std::string output_key)
    : TaskComposerTask("AddTwoNumbers", AddTaskComposerNode::ports(), false)
  {
    input_keys_.add(INPUT_LEFT_PORT, std::move(left_key));
    input_keys_.add(INPUT_RIGHT_PORT, std::move(right_key));
    output_keys_.add(OUTPUT_RESULT_PORT, std::move(output_key));
    validatePorts();
  }

  static TaskComposerNodePorts ports()
  {
    TaskComposerNodePorts ports;
    ports.input_required[INPUT_LEFT_PORT] = TaskComposerNodePorts::SINGLE;
    ports.input_required[INPUT_RIGHT_PORT] = TaskComposerNodePorts::SINGLE;
    ports.output_required[OUTPUT_RESULT_PORT] = TaskComposerNodePorts::SINGLE;
    return ports;
  }

  TaskComposerNodeInfo runImpl(TaskComposerContext& context,
                               OptionalTaskComposerExecutor /*executor*/) const override final
  {
    TaskComposerNodeInfo info(*this);
    info.return_value = 0;
    std::cout << name_ << "\n";
    double result = getData(context, INPUT_LEFT_PORT).as<double>() + getData(context, INPUT_RIGHT_PORT).as<double>();
    setData(context, OUTPUT_RESULT_PORT, result);
    return info;
  }
};

const std::string AddTaskComposerNode::INPUT_LEFT_PORT = "left";
const std::string AddTaskComposerNode::INPUT_RIGHT_PORT = "right";
const std::string AddTaskComposerNode::OUTPUT_RESULT_PORT = "result";

class MultiplyTaskComposerNode : public TaskComposerTask
{
public:
  // Requried
  static const std::string INPUT_LEFT_PORT;
  static const std::string INPUT_RIGHT_PORT;
  static const std::string OUTPUT_RESULT_PORT;

  MultiplyTaskComposerNode(std::string left_key, std::string right_key, std::string output_key)
    : TaskComposerTask("MultiplyTwoNumbers", MultiplyTaskComposerNode::ports(), false)
  {
    input_keys_.add(INPUT_LEFT_PORT, std::move(left_key));
    input_keys_.add(INPUT_RIGHT_PORT, std::move(right_key));
    output_keys_.add(OUTPUT_RESULT_PORT, std::move(output_key));
    validatePorts();
  }

  static TaskComposerNodePorts ports()
  {
    TaskComposerNodePorts ports;
    ports.input_required[INPUT_LEFT_PORT] = TaskComposerNodePorts::SINGLE;
    ports.input_required[INPUT_RIGHT_PORT] = TaskComposerNodePorts::SINGLE;
    ports.output_required[OUTPUT_RESULT_PORT] = TaskComposerNodePorts::SINGLE;
    return ports;
  }

  TaskComposerNodeInfo runImpl(TaskComposerContext& context,
                               OptionalTaskComposerExecutor /*executor*/) const override final
  {
    TaskComposerNodeInfo info(*this);
    info.return_value = 0;
    std::cout << name_ << "\n";
    double result = getData(context, INPUT_LEFT_PORT).as<double>() * getData(context, INPUT_RIGHT_PORT).as<double>();
    setData(context, OUTPUT_RESULT_PORT, result);
    return info;
  }
};

const std::string MultiplyTaskComposerNode::INPUT_LEFT_PORT = "left";
const std::string MultiplyTaskComposerNode::INPUT_RIGHT_PORT = "right";
const std::string MultiplyTaskComposerNode::OUTPUT_RESULT_PORT = "result";

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

  tesseract::common::GeneralResourceLocator locator;
  auto resource = locator.locateResource("package://tesseract_task_composer/config/task_composer_plugins.yaml");
  std::filesystem::path config_path(resource->getFilePath());
  TaskComposerPluginFactory factory(config_path, *resource);

  auto task_executor = factory.createTaskComposerExecutor("TaskflowExecutor");
  auto context = std::make_shared<TaskComposerContext>(task_composer.getName(), std::move(task_data));
  TaskComposerFuture::UPtr future = task_executor->run(task_composer, std::move(context));
  future->wait();

  std::cout << "Output: " << future->context->data_storage->getData("task3_output").as<double>() << "\n";
}
