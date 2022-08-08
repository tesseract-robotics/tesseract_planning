
#include <iostream>
#include <tesseract_common/utils.h>
#include <tesseract_task_composer/task_composer_graph.h>
#include <taskflow/taskflow.hpp>

using namespace tesseract_planning;

class AddTaskComposerNode : public TaskComposerNode
{
public:
  AddTaskComposerNode(std::string left_key, std::string right_key, std::string output_key)
    : TaskComposerNode("AddTwoNumbers")
    , left_key_(std::move(left_key))
    , right_key_(std::move(right_key))
    , output_key_(std::move(output_key))
  {
  }

  int run(TaskComposerInput& input) const override final
  {
    std::cout << name_ << std::endl;
    double result =
        input.data_storage->getData(left_key_).as<double>() + input.data_storage->getData(right_key_).as<double>();
    input.data_storage->setData(output_key_, result);
    return 0;
  }

protected:
  std::string left_key_;
  std::string right_key_;
  std::string output_key_;
};

class MultiplyTaskComposerNode : public TaskComposerNode
{
public:
  MultiplyTaskComposerNode(std::string left_key, std::string right_key, std::string output_key)
    : TaskComposerNode("MultiplyTwoNumbers")
    , left_key_(std::move(left_key))
    , right_key_(std::move(right_key))
    , output_key_(std::move(output_key))
  {
  }

  int run(TaskComposerInput& input) const override final
  {
    std::cout << name_ << std::endl;
    double result =
        input.data_storage->getData(left_key_).as<double>() * input.data_storage->getData(right_key_).as<double>();
    input.data_storage->setData(output_key_, result);
    return 0;
  }

protected:
  std::string left_key_;
  std::string right_key_;
  std::string output_key_;
};

std::unique_ptr<tf::Taskflow> convertToTaskflow(TaskComposerGraph& task_composer, TaskComposerInput::Ptr task_input)
{
  auto taskflow = std::make_unique<tf::Taskflow>(task_composer.getName());

  // Generate process tasks for each node
  std::vector<tf::Task> tasks;
  const auto& nodes = task_composer.getNodes();
  tasks.reserve(nodes.size());
  for (const auto& node : nodes)
  {
    auto edges = node->getEdges();
    if (edges.size() > 1 && node->getType() == TaskComposerNodeType::CONDITIONAL_TASK)
      tasks.push_back(taskflow->emplace([node, task_input] { return node->run(*task_input); }).name(node->getName()));
    else
      tasks.push_back(taskflow->emplace([node, task_input] { node->run(*task_input); }).name(node->getName()));
  }

  for (std::size_t i = 0; i < nodes.size(); ++i)
  {
    // Ensure the current task precedes the tasks that it is connected to
    const auto& node = nodes[i];
    auto edges = node->getEdges();
    for (int idx : edges)
    {
      if (idx >= 0)
        tasks.at(i).precede(tasks.at(static_cast<std::size_t>(idx)));
      else
        throw std::runtime_error("Invalid TaskComposerPipeline: Node specified with invalid edge");
    }
  }

  return taskflow;
}

int main()
{
  double a{ 1 };
  double b{ 3 };
  double c{ 5 };
  double d{ 9 };
  auto task_data = std::make_shared<TaskComposerDataStorage>();
  task_data->setData("a", a);
  task_data->setData("b", b);
  task_data->setData("c", c);
  task_data->setData("d", d);

  auto task_input = std::make_shared<TaskComposerInput>(task_data);

  // result = a * (b + c) + d
  auto task1 = std::make_unique<AddTaskComposerNode>("b", "c", "task1_output");
  auto task2 = std::make_unique<MultiplyTaskComposerNode>("a", "task1_output", "task2_output");
  auto task3 = std::make_unique<AddTaskComposerNode>("task2_output", "d", "task3_output");

  TaskComposerGraph task_composer;
  int task1_id = task_composer.addNode(std::move(task1));
  int task2_id = task_composer.addNode(std::move(task2));
  int task3_id = task_composer.addNode(std::move(task3));
  task_composer.addEdges(task1_id, { task2_id });
  task_composer.addEdges(task2_id, { task3_id });

  std::unique_ptr<tf::Taskflow> taskflow = convertToTaskflow(task_composer, task_input);

  std::ofstream out_data;
  out_data.open(tesseract_common::getTempPath() + "task_composer_example.dot");
  taskflow->dump(out_data);  // dump the graph including dynamic tasks
  out_data.close();

  tf::Executor executor;
  executor.run(*taskflow);
  executor.wait_for_all();

  std::cout << "Output: " << task_data->getData("task3_output").as<double>() << std::endl;
}
