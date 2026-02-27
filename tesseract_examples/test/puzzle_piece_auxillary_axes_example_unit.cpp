#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_examples/puzzle_piece_auxillary_axes_example.h>
#include <filesystem>
#include <tesseract/environment/environment.h>
#include <tesseract/common/resource_locator.h>

using namespace tesseract::examples;
using namespace tesseract::common;
using namespace tesseract::environment;

TEST(TesseractExamples, PuzzlePieceAuxillaryAxesCppTrajOptExampleUnit)  // NOLINT
{
  auto locator = std::make_shared<GeneralResourceLocator>();
  std::filesystem::path urdf_path =
      locator->locateResource("package://tesseract/support/urdf/puzzle_piece_workcell.urdf")->getFilePath();
  std::filesystem::path srdf_path =
      locator->locateResource("package://tesseract/support/urdf/puzzle_piece_workcell.srdf")->getFilePath();
  auto env = std::make_shared<Environment>();
  if (!env->init(urdf_path, srdf_path, locator))
    exit(1);

  PuzzlePieceAuxillaryAxesExample example(env, nullptr, false, false);
  EXPECT_TRUE(example.run());
}

TEST(TesseractExamples, PuzzlePieceAuxillaryAxesCppTrajOptIfoptExampleUnit)  // NOLINT
{
  auto locator = std::make_shared<GeneralResourceLocator>();
  std::filesystem::path urdf_path =
      locator->locateResource("package://tesseract/support/urdf/puzzle_piece_workcell.urdf")->getFilePath();
  std::filesystem::path srdf_path =
      locator->locateResource("package://tesseract/support/urdf/puzzle_piece_workcell.srdf")->getFilePath();
  auto env = std::make_shared<Environment>();
  if (!env->init(urdf_path, srdf_path, locator))
    exit(1);

  PuzzlePieceAuxillaryAxesExample example(env, nullptr, true, false);
  EXPECT_TRUE(example.run());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
