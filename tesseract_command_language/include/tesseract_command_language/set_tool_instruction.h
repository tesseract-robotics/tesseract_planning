/**
 * @file set_tool_instruction.h
 * @brief Set tool ID
 *
 * @author Levi Armstrong
 * @date March 11, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#ifndef TESSERACT_COMMAND_LANGUAGE_SET_TOOL_INSTRUCTION_H
#define TESSERACT_COMMAND_LANGUAGE_SET_TOOL_INSTRUCTION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <boost/uuid/uuid.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/poly/instruction_poly.h>

namespace tesseract_planning
{
class SetToolInstruction final : public InstructionInterface
{
public:
  SetToolInstruction() = default;  // Required for boost serialization do not use
  SetToolInstruction(int tool_id);

  // Instruction

  /**
   * @brief Get the UUID
   * @return The UUID
   */
  const boost::uuids::uuid& getUUID() const override final;
  /**
   * @brief Set the UUID
   * @param uuid The UUID
   */
  void setUUID(const boost::uuids::uuid& uuid) override final;
  /**
   * @brief Regenerate the UUID
   */
  void regenerateUUID() override final;

  /**
   * @brief Get the parent UUID
   * @return The parent UUID
   */
  const boost::uuids::uuid& getParentUUID() const override final;
  /**
   * @brief Set the parent UUID
   * @param uuid The parent UUID
   */
  void setParentUUID(const boost::uuids::uuid& uuid) override final;

  /**
   * @brief Get the description
   * @return The description
   */
  const std::string& getDescription() const override final;
  /**
   * @brief Set the description
   * @param description The description
   */
  void setDescription(const std::string& description) override final;

  /**
   * @brief Output the contents to std::cout
   * @param prefix The prefix to add to each variable
   */
  void print(const std::string& prefix = "") const override final;  // NOLINT

  /**
   * @brief Make a deep copy of the object
   * @return A deep copy
   */
  std::unique_ptr<InstructionInterface> clone() const override final;

  // SetToolInstruction

  /**
   * @brief Get the tool ID
   * @return The tool ID
   */
  int getTool() const;

private:
  /** @brief The instructions UUID */
  boost::uuids::uuid uuid_{};
  /** @brief The parent UUID if created from createChild */
  boost::uuids::uuid parent_uuid_{};
  /** @brief The description of the instruction */
  std::string description_{ "Tesseract Set Tool Instruction" };
  /** @brief The tool ID */
  int tool_id_{ -1 };

  /**
   * @brief Check if two objects are equal
   * @param other The other object to compare with
   * @return True if equal, otherwise false
   */
  bool equals(const InstructionInterface& other) const override final;

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::SetToolInstruction)
BOOST_CLASS_TRACKING(tesseract_planning::SetToolInstruction, boost::serialization::track_never)

#endif  // TESSERACT_COMMAND_LANGUAGE_SET_TOOL_INSTRUCTION_H
