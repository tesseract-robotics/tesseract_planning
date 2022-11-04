/**
 * @file profile_dictionary_tests.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date December 2, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/profile_dictionary.h>

struct ProfileBase
{
  using ConstPtr = std::shared_ptr<const ProfileBase>;
  int a{ 0 };
};

struct ProfileTest : public ProfileBase
{
  ProfileTest() = default;
  ProfileTest(int a) { this->a = a; }
};

struct ProfileBase2
{
  using ConstPtr = std::shared_ptr<const ProfileBase2>;
  int b{ 0 };
};

struct ProfileTest2 : public ProfileBase2
{
  ProfileTest2() = default;
  ProfileTest2(int b) { this->b = b; }
};

using namespace tesseract_planning;

TEST(TesseractPlanningProfileDictionaryUnit, ProfileDictionaryTest)  // NOLINT
{
  ProfileDictionary profiles;

  EXPECT_THROW(profiles.getProfile<ProfileBase>("ns", "key"), std::runtime_error);  // NOLINT

  profiles.addProfile<ProfileBase>("ns", "key", std::make_shared<ProfileTest>());

  ProfileBase::ConstPtr profile;
  ASSERT_NO_THROW(profile = profiles.getProfile<ProfileBase>("ns", "key"));  // NOLINT

  ASSERT_TRUE(profile != nullptr);
  EXPECT_EQ(profile->a, 0);

  // Check add same profile with different key
  profiles.addProfile<ProfileBase>("ns", "key2", profile);
  ProfileBase::ConstPtr profile2;
  ASSERT_NO_THROW(profile2 = profiles.getProfile<ProfileBase>("ns", "key2"));  // NOLINT
  ASSERT_TRUE(profile2 != nullptr);
  EXPECT_EQ(profile2->a, 0);

  // Check replacing a profile
  profiles.addProfile<ProfileBase>("ns", "key", std::make_shared<ProfileTest>(10));
  ProfileBase::ConstPtr profile_check;
  ASSERT_NO_THROW(profile_check = profiles.getProfile<ProfileBase>("ns", "key"));  // NOLINT
  ASSERT_TRUE(profile_check != nullptr);
  EXPECT_EQ(profile_check->a, 10);

  auto profile_map = profiles.getProfileEntry<ProfileBase>("ns");
  auto it = profile_map.find("key");
  EXPECT_TRUE(it != profile_map.end());
  EXPECT_EQ(it->second->a, 10);

  profiles.addProfile<ProfileBase>("ns", "key", std::make_shared<ProfileTest>(20));
  ProfileBase::ConstPtr profile_check2;
  ASSERT_NO_THROW(profile_check2 = profiles.getProfile<ProfileBase>("ns", "key"));  // NOLINT
  ASSERT_TRUE(profile_check2 != nullptr);
  EXPECT_EQ(profile_check2->a, 20);

  // Request a profile entry namespace that does not exist
  EXPECT_ANY_THROW(profiles.getProfile<int>("DoesNotExist", "DoesNotExist"));  // NOLINT

  // Request a profile that does not exist
  EXPECT_ANY_THROW(profiles.getProfile<ProfileBase>("DoesNotExist", "key"));  // NOLINT

  // Request a profile that does not exist
  EXPECT_ANY_THROW(profiles.getProfile<ProfileBase>("ns", "DoesNotExist"));  // NOLINT

  // Check adding a empty namespace
  EXPECT_ANY_THROW(profiles.addProfile<ProfileBase>("", "key3", nullptr));  // NOLINT

  // Check adding a empty key
  EXPECT_ANY_THROW(profiles.addProfile<ProfileBase>("ns", "", nullptr));  // NOLINT

  // Check adding a nullptr profile
  EXPECT_ANY_THROW(profiles.addProfile<ProfileBase>("ns", "key", nullptr));  // NOLINT

  // Add different profile entry
  profiles.addProfile<ProfileBase2>("ns", "key", std::make_shared<ProfileTest2>(5));
  EXPECT_TRUE(profiles.hasProfileEntry<ProfileBase2>("ns"));
  ProfileBase2::ConstPtr profile_check3;
  ASSERT_NO_THROW(profile_check3 = profiles.getProfile<ProfileBase2>("ns", "key"));  // NOLINT
  ASSERT_TRUE(profile_check3 != nullptr);
  EXPECT_EQ(profile_check3->b, 5);
  // Check that other profile entry with same key is not affected
  ProfileBase::ConstPtr profile_check4;
  ASSERT_NO_THROW(profile_check4 = profiles.getProfile<ProfileBase>("ns", "key"));  // NOLINT
  ASSERT_TRUE(profile_check4 != nullptr);
  EXPECT_EQ(profile_check4->a, 20);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
