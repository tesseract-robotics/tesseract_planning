/**
 * @file ompl_collision_threads_unit.cpp
 * @brief One OMPL collision checker shared by several threads gives each thread its own result
 *
 * @author Jelle Feringa
 * @date November 7, 2026
 *
 * @copyright Copyright (c) 2026, Jelle Feringa
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <gtest/gtest.h>
#include <atomic>
#include <cmath>
#include <filesystem>
#include <functional>
#include <memory>
#include <thread>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/resource_locator.h>
#include <tesseract/collision/types.h>
#include <tesseract/environment/environment.h>
#include <tesseract/environment/commands/add_link_command.h>
#include <tesseract/geometry/impl/box.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/scene_graph/joint.h>
#include <tesseract/scene_graph/link.h>

#include <tesseract/motion_planners/ompl/collision_cost_objective.h>
#include <tesseract/motion_planners/ompl/continuous_motion_validator.h>
#include <tesseract/motion_planners/ompl/state_collision_validator.h>
#include <tesseract/motion_planners/ompl/utils.h>

using namespace tesseract::motion_planners;

namespace
{
/** @brief Threads that call one shared checker at once */
constexpr std::size_t NUM_THREADS{ 4 };

/** @brief Calls each thread makes */
constexpr int NUM_EVALUATIONS{ 100 };

/** @brief Index of joint_a2, the shoulder, which leans the arm towards (+) or away from (-) the box */
constexpr unsigned SHOULDER{ 1 };

/** @brief Index of joint_a1, the base, swept by the motions */
constexpr unsigned BASE{ 0 };

/** @brief Shoulder angle (rad) of the even threads' states: leaning away from the box, collision free */
constexpr double FREE_SHOULDER{ -0.6 };

/** @brief Shoulder angle (rad) of thread 0 in the cost test; thread i leans SHOULDER_STEP further into the box */
constexpr double COLLIDING_SHOULDER{ 0.9 };

/** @brief Extra shoulder angle (rad) per thread, so each thread penetrates the box to its own depth */
constexpr double SHOULDER_STEP{ 0.1 };

/** @brief Base angle (rad) at each end of a motion */
constexpr double MOTION_BASE{ 0.3 };

/** @brief Edge length (m) of the box the arm leans into; its near face is BOX_FACE_X from the base */
constexpr double BOX_SIZE{ 1.0 };

/** @brief Distance (m) from the robot base to the box's near face along x */
constexpr double BOX_FACE_X{ 0.5 };

/** @brief Height (m) of the box's centre */
constexpr double BOX_CENTER_Z{ 0.5 };

/** @brief Tolerance on a thread's result against its single-threaded reference */
constexpr double RESULT_TOL{ 1e-9 };

/**
 * @brief Call one shared checker from NUM_THREADS threads at once and compare against a single-threaded run.
 * @param evaluate Runs the checker once with thread i's input and returns its result
 */
void runShared(const std::function<double(std::size_t)>& evaluate)
{
  std::vector<double> reference(NUM_THREADS);
  for (std::size_t i = 0; i < NUM_THREADS; ++i)
    reference[i] = evaluate(i);

  // Each thread must get a result its neighbour doesn't, or a result computed at another thread's state goes unnoticed
  for (std::size_t i = 1; i < NUM_THREADS; ++i)
    ASSERT_GT(std::abs(reference[i] - reference[i - 1]), RESULT_TOL) << "threads " << i - 1 << " and " << i << " agree";

  std::vector<int> mismatches(NUM_THREADS, 0);
  std::atomic<std::size_t> ready{ 0 };
  std::vector<std::thread> threads;
  threads.reserve(NUM_THREADS);
  for (std::size_t i = 0; i < NUM_THREADS; ++i)
  {
    threads.emplace_back([&, i] {
      // Start together, so the calls overlap
      ++ready;
      while (ready.load() < NUM_THREADS)
        std::this_thread::yield();

      for (int k = 0; k < NUM_EVALUATIONS; ++k)
      {
        if (std::abs(evaluate(i) - reference[i]) > RESULT_TOL)
          ++mismatches[i];
      }
    });
  }
  for (auto& thread : threads)
    thread.join();

  EXPECT_EQ(mismatches, std::vector<int>(NUM_THREADS, 0));
}

/** @brief A collision free shoulder angle for the even threads, a colliding one for the odd threads */
double alternatingShoulder(std::size_t i) { return (i % 2 == 0) ? FREE_SHOULDER : COLLIDING_SHOULDER; }

/** @brief Add a BOX_SIZE box whose near face is BOX_FACE_X in front of the robot base */
void addBox(tesseract::environment::Environment& env)
{
  tesseract::scene_graph::Link link("threads_box");
  auto collision = std::make_shared<tesseract::scene_graph::Collision>();
  collision->origin = Eigen::Isometry3d::Identity();
  collision->origin.translation() = Eigen::Vector3d(BOX_FACE_X + (BOX_SIZE / 2.0), 0, BOX_CENTER_Z);
  collision->geometry = std::make_shared<tesseract::geometry::Box>(BOX_SIZE, BOX_SIZE, BOX_SIZE);
  link.collision.push_back(collision);

  tesseract::scene_graph::Joint joint("threads_box_joint");
  joint.parent_link_id = "base_link";
  joint.child_link_id = link.getId();
  joint.type = tesseract::scene_graph::JointType::FIXED;

  ASSERT_TRUE(env.applyCommand(std::make_shared<tesseract::environment::AddLinkCommand>(link, joint)));
}
}  // namespace

class OMPLCollisionThreadsTest : public testing::Test
{
public:
  std::shared_ptr<tesseract::environment::Environment> env = std::make_shared<tesseract::environment::Environment>();
  std::shared_ptr<const tesseract::kinematics::JointGroup> manip;
  std::shared_ptr<ompl::base::RealVectorStateSpace> rss = std::make_shared<ompl::base::RealVectorStateSpace>();
  ompl::base::SpaceInformationPtr si;
  OMPLStateExtractor extractor;
  tesseract::collision::ContactManagerConfig contact_config;

  void SetUp() override
  {
    auto locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
    const std::filesystem::path urdf_path(
        locator->locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf")->getFilePath());
    const std::filesystem::path srdf_path(
        locator->locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.srdf")->getFilePath());
    ASSERT_TRUE(env->init(urdf_path, srdf_path, locator));
    addBox(*env);

    manip = env->getJointGroup("manipulator");
    const auto dof = static_cast<unsigned>(manip->numJoints());
    const std::vector<tesseract::common::JointId> joint_ids = manip->getJointIds();
    const Eigen::MatrixX2d limits = manip->getLimits().joint_limits;
    for (unsigned j = 0; j < dof; ++j)
      rss->addDimension(joint_ids[j].name(), limits(j, 0), limits(j, 1));

    si = std::make_shared<ompl::base::SpaceInformation>(rss);
    si->setup();

    extractor = [dof](const ompl::base::State* state) -> Eigen::Map<Eigen::VectorXd> {
      return RealVectorStateSpaceExtractor(state, dof);
    };
  }

  /** @brief A state with every joint at zero except the shoulder and base */
  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> makeState(double shoulder, double base) const
  {
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state(rss);
    for (unsigned j = 0; j < rss->getDimension(); ++j)
      state->values[j] = 0.0;
    state->values[SHOULDER] = shoulder;
    state->values[BASE] = base;
    return state;
  }
};

TEST_F(OMPLCollisionThreadsTest, StateCollisionValidatorSharedAcrossThreads)  // NOLINT
{
  StateCollisionValidator validator(si, *env, manip, contact_config, extractor);
  runShared([&](std::size_t i) {
    const auto state = makeState(alternatingShoulder(i), 0.0);
    return validator.isValid(state.get()) ? 1.0 : 0.0;
  });
}

TEST_F(OMPLCollisionThreadsTest, CollisionCostObjectiveSharedAcrossThreads)  // NOLINT
{
  CollisionCostObjective objective(si, *env, manip, contact_config, extractor);
  runShared([&](std::size_t i) {
    const auto state = makeState(COLLIDING_SHOULDER + (SHOULDER_STEP * static_cast<double>(i)), 0.0);
    return objective.stateCost(state.get()).value();
  });
}

TEST_F(OMPLCollisionThreadsTest, ContinuousMotionValidatorSharedAcrossThreads)  // NOLINT
{
  // No state validator, so the continuous check alone decides each motion
  ContinuousMotionValidator validator(si, nullptr, *env, manip, contact_config, extractor);
  runShared([&](std::size_t i) {
    const auto start = makeState(alternatingShoulder(i), -MOTION_BASE);
    const auto end = makeState(alternatingShoulder(i), MOTION_BASE);
    return validator.checkMotion(start.get(), end.get()) ? 1.0 : 0.0;
  });
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
