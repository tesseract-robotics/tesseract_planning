#include <taskflow/taskflow.hpp>
#include <tesseract_common/utils.h>

struct TaskInput
{
  int* cnt;
};

/**
 * @brief This example is to explore the dynamic tasking functionality of taskflow.
 * @details The first tasks will change a variable which is then used by a subsequent task to dynamically create n
 * number of tasks.
 * @return
 */
int main(int /*argc*/, char** /*argv*/)
{
  tf::Executor executor;
  tf::Taskflow taskflow;

  // launch an asynchronous task from a running task
  taskflow.emplace([&]() {
    tf::Taskflow async_taskflow;
    auto [A, B, C, D] = async_taskflow.emplace(  // create four tasks
        []() { std::cout << "TaskA\n"; },
        []() { std::cout << "TaskB\n"; },
        []() { std::cout << "TaskC\n"; },
        []() { std::cout << "TaskD\n"; });

    A.precede(B, C);  // A runs before B and C
    D.succeed(B, C);  // D runs after  B and C
    executor.run(async_taskflow).wait();
  });

  executor.run(taskflow).wait();  // run the taskflow to spawn subflows

  std::ofstream out_data;
  out_data.open(tesseract_common::getTempPath() + "async_taskflow_example-" + tesseract_common::getTimestampString() +
                ".dot");
  taskflow.dump(out_data);  // dump the graph including dynamic tasks
  out_data.close();
}
