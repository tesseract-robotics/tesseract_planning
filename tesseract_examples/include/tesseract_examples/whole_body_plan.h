#ifndef TESSERACT_WHOLE_BODY_PLAN_EXAMPLE_H
#define TESSERACT_WHOLE_BODY_PLAN_EXAMPLE_H

#include <tesseract_examples/example.h>

namespace tesseract_examples
{
/**
 * @brief Basic example leveraging trajopt and tesseract for cartesian planning
 */
class WholeBodyPlan : public Example
{
public:
  WholeBodyPlan(std::shared_ptr<tesseract_environment::Environment> env,
                        std::shared_ptr<tesseract_visualization::Visualization> plotter = nullptr,
                        bool ifopt = false,
                        bool debug = false);

  ~WholeBodyPlan() override = default;
  WholeBodyPlan(const WholeBodyPlan&) = default;
  WholeBodyPlan& operator=(const WholeBodyPlan&) = default;
  WholeBodyPlan(WholeBodyPlan&&) = default;
  WholeBodyPlan& operator=(WholeBodyPlan&&) = default;

  bool run() override final;

private:
  bool ifopt_;
  bool debug_;
};

}  // namespace tesseract_examples

#endif  // TESSERACT_ROS_EXAMPLES_BASIC_CARTESIAN_EXAMPLE_H
