#ifndef TESSERACT_TASK_COMPOSER_SERIALIZATION_UTILS_HPP
#define TESSERACT_TASK_COMPOSER_SERIALIZATION_UTILS_HPP

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/serialization/unique_ptr.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <fstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/serialization.h>
#include <tesseract_common/utils.h>

namespace tesseract_planning::test_suite
{
template <typename T>
inline void runSerializationTest(const T& input, const std::string& file_name)
{
  const std::string filepath = tesseract_common::getTempPath() + file_name + ".xml";
  tesseract_common::Serialization::toArchiveFileXML<T>(input, filepath);
  auto ninput = tesseract_common::Serialization::fromArchiveFileXML<T>(filepath);
  EXPECT_FALSE(input != ninput);
}

template <typename T>
inline void runSerializationPointerTest(const T& input, const std::string& file_name)
{
  const std::string filepath = tesseract_common::getTempPath() + file_name + ".xml";
  tesseract_common::Serialization::toArchiveFileXML<T>(input, filepath);
  auto ninput = tesseract_common::Serialization::fromArchiveFileXML<T>(filepath);
  EXPECT_FALSE(*input != *ninput);
}
}  // namespace tesseract_planning::test_suite
#endif  // TESSERACT_TASK_COMPOSER_SERIALIZATION_UTILS_HPP
