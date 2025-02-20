#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_examples/car_seat_example.h>
#include <filesystem>
#include <tesseract_environment/environment.h>
#include <tesseract_common/resource_locator.h>

using namespace tesseract_examples;
using namespace tesseract_common;
using namespace tesseract_environment;

TEST(TesseractExamples, CarSeatCppTrajOptExampleUnit)  // NOLINT
{
  auto locator = std::make_shared<GeneralResourceLocator>();
  std::filesystem::path urdf_path =
      locator->locateResource("package://tesseract_support/urdf/car_seat_demo.urdf")->getFilePath();
  std::filesystem::path srdf_path =
      locator->locateResource("package://tesseract_support/urdf/car_seat_demo.srdf")->getFilePath();
  auto env = std::make_shared<Environment>();
  if (!env->init(urdf_path, srdf_path, locator))
    exit(1);

  CarSeatExample example(env, nullptr, false, false);
  EXPECT_TRUE(example.run());
}

TEST(TesseractExamples, CarSeatCppTrajOptIfoptExampleUnit)  // NOLINT
{
  auto locator = std::make_shared<GeneralResourceLocator>();
  std::filesystem::path urdf_path =
      locator->locateResource("package://tesseract_support/urdf/car_seat_demo.urdf")->getFilePath();
  std::filesystem::path srdf_path =
      locator->locateResource("package://tesseract_support/urdf/car_seat_demo.srdf")->getFilePath();
  auto env = std::make_shared<Environment>();
  if (!env->init(urdf_path, srdf_path, locator))
    exit(1);

  CarSeatExample example(env, nullptr, true, false);
  EXPECT_TRUE(example.run());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
