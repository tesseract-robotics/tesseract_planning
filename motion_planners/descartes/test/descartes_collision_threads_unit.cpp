/**
 * @file descartes_collision_threads_unit.cpp
 * @brief One Descartes collision edge evaluator shared by several threads gives each thread its own result
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
#include <descartes_light/types.h>
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

#include <tesseract/motion_planners/descartes/descartes_collision_edge_evaluator.h>

using namespace tesseract::motion_planners;
using tesseract::collision::CollisionEvaluatorType;

namespace
{
/** @brief Threads that call one shared evaluator at once */
constexpr std::size_t NUM_THREADS{ 4 };

/** @brief Calls each thread makes */
constexpr int NUM_EVALUATIONS{ 100 };

/** @brief Index of joint_a2, the shoulder, which leans the arm into the box */
constexpr Eigen::Index SHOULDER{ 1 };

/** @brief Index of joint_a1, the base, swept along each edge */
constexpr Eigen::Index BASE{ 0 };

/** @brief Shoulder angle (rad) of thread 0, leaning into the box */
constexpr double COLLIDING_SHOULDER{ 0.9 };

/** @brief Extra shoulder angle (rad) per thread, so each thread penetrates the box to its own depth */
constexpr double SHOULDER_STEP{ 0.1 };

/** @brief Base angle (rad) at each end of an edge */
constexpr double EDGE_BASE{ 0.3 };

/** @brief Longest valid segment length (m) for the LVS types */
constexpr double LONGEST_VALID_SEGMENT_LENGTH{ 0.05 };

/** @brief Edge length (m) of the box the arm leans into */
constexpr double BOX_SIZE{ 1.0 };

/** @brief Distance (m) from the robot base to the box's near face along x */
constexpr double BOX_FACE_X{ 0.5 };

/** @brief Height (m) of the box's centre */
constexpr double BOX_CENTER_Z{ 0.5 };

/** @brief Tolerance on a thread's cost against its single-threaded reference */
constexpr double RESULT_TOL{ 1e-9 };

/**
 * @brief Call one shared evaluator from NUM_THREADS threads at once and compare against a single-threaded run.
 * @param evaluate Runs the evaluator once with thread i's edge and returns its cost
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

class DescartesCollisionThreadsTest : public testing::TestWithParam<CollisionEvaluatorType>
{
public:
  std::shared_ptr<tesseract::environment::Environment> env = std::make_shared<tesseract::environment::Environment>();
  std::shared_ptr<const tesseract::kinematics::JointGroup> manip;

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
  }

  /** @brief A state with every joint at zero except the shoulder and base */
  descartes_light::State<double>::ConstPtr makeState(double shoulder, double base) const
  {
    Eigen::VectorXd values = Eigen::VectorXd::Zero(manip->numJoints());
    values(SHOULDER) = shoulder;
    values(BASE) = base;
    return std::make_shared<const descartes_light::State<double>>(values);
  }
};

TEST_P(DescartesCollisionThreadsTest, EdgeEvaluatorSharedAcrossThreads)  // NOLINT
{
  tesseract::collision::CollisionCheckConfig check_config;
  check_config.type = GetParam();
  check_config.longest_valid_segment_length = LONGEST_VALID_SEGMENT_LENGTH;

  // allow_collision, so an edge in collision returns its cost instead of only being rejected
  const DescartesCollisionEdgeEvaluatorD evaluator(
      *env, manip, tesseract::collision::ContactManagerConfig(), check_config, true);
  runShared([&](std::size_t i) {
    const double shoulder = COLLIDING_SHOULDER + (SHOULDER_STEP * static_cast<double>(i));
    const auto start = makeState(shoulder, -EDGE_BASE);
    const auto end = makeState(shoulder, EDGE_BASE);
    return evaluator.evaluate(*start, *end).second;
  });
}

INSTANTIATE_TEST_SUITE_P(CollisionTypes,
                         DescartesCollisionThreadsTest,
                         testing::Values(CollisionEvaluatorType::DISCRETE,
                                         CollisionEvaluatorType::LVS_DISCRETE,
                                         CollisionEvaluatorType::CONTINUOUS,
                                         CollisionEvaluatorType::LVS_CONTINUOUS));

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
