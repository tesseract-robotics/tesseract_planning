/**
 * @file instruction.h
 * @brief
 *
 * @author Levi Armstrong
 * @date June 15, 2020
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
#ifndef TESSERACT_COMMAND_LANGUAGE_INSTRUCTION_H
#define TESSERACT_COMMAND_LANGUAGE_INSTRUCTION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <typeindex>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/unique_ptr.hpp>
#include <boost/serialization/export.hpp>
#include <boost/type_traits/is_virtual_base_of.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/core/waypoint.h>
#include <tesseract_command_language/core/serialization.h>
#include <tesseract_common/sfinae_utils.h>

#ifdef SWIG
%ignore std::vector<tesseract_planning::Instruction>::vector(size_type);
%ignore std::vector<tesseract_planning::Instruction>::resize(size_type);
%ignore tesseract_planning::Instruction::getType;
%pythondynamic tesseract_planning::Instruction;
#endif  // SWIG

/** @brief If shared library, this must go in the header after the class definition */
#define TESSERACT_INSTRUCTION_EXPORT_KEY(inst)                                                                         \
  BOOST_CLASS_EXPORT_KEY2(tesseract_planning::detail_instruction::InstructionInner<inst>, #inst)                       \
  BOOST_CLASS_TRACKING(tesseract_planning::detail_instruction::InstructionInner<inst>,                                 \
                       boost::serialization::track_never)

/** @brief If shared library, this must go in the cpp after the implicit instantiation of the serialize function */
#define TESSERACT_INSTRUCTION_EXPORT_IMPLEMENT(inst)                                                                   \
  BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::detail_instruction::InstructionInner<inst>)

/**
 * @brief This should not be used within shared libraries use the two above.
 * If not in a shared library it can go in header or cpp
 */
#define TESSERACT_INSTRUCTION_EXPORT(inst)                                                                             \
  TESSERACT_INSTRUCTION_EXPORT_KEY(inst)                                                                               \
  TESSERACT_INSTRUCTION_EXPORT_IMPLEMENT(inst)

namespace tesseract_planning
{
#ifndef SWIG
namespace detail_instruction
{
CREATE_MEMBER_CHECK(getDescription);
CREATE_MEMBER_CHECK(setDescription);
CREATE_MEMBER_CHECK(print);
CREATE_MEMBER_FUNC_SIGNATURE_NOARGS_CHECK(getDescription, const std::string&);
CREATE_MEMBER_FUNC_SIGNATURE_CHECK(setDescription, void, const std::string&);
CREATE_MEMBER_FUNC_SIGNATURE_CHECK(print, void, std::string);

struct InstructionInnerBase
{
  InstructionInnerBase() = default;
  virtual ~InstructionInnerBase() = default;
  InstructionInnerBase(const InstructionInnerBase&) = delete;
  InstructionInnerBase& operator=(const InstructionInnerBase&) = delete;
  InstructionInnerBase(InstructionInnerBase&&) = delete;
  InstructionInnerBase& operator=(InstructionInnerBase&&) = delete;

  // User-defined methods
  virtual const std::string& getDescription() const = 0;

  virtual void setDescription(const std::string& description) = 0;

  virtual void print(const std::string& prefix) const = 0;

  virtual bool operator==(const InstructionInnerBase& rhs) const = 0;

  // This is not required for user defined implementation
  virtual bool operator!=(const InstructionInnerBase& rhs) const = 0;

  // This is not required for user defined implementation
  virtual std::type_index getType() const = 0;

  // This is not required for user defined implementation
  virtual void* recover() = 0;

  // This is not required for user defined implementation
  virtual const void* recover() const = 0;

  // This is not required for user defined implementation
  virtual std::unique_ptr<InstructionInnerBase> clone() const = 0;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& /*ar*/, const unsigned int /*version*/)  // NOLINT
  {
  }
};

template <typename T>
struct InstructionInner final : InstructionInnerBase
{
  InstructionInner()
  {
    static_assert(has_member_getDescription<T>::value, "Class does not have member function 'getDescription'");
    static_assert(has_member_setDescription<T>::value, "Class does not have member function 'setDescription'");
    static_assert(has_member_print<T>::value, "Class does not have member function 'print'");
    static_assert(has_member_func_signature_getDescription<T>::value,
                  "Class 'getDescription' function has incorrect signature");
    static_assert(has_member_func_signature_setDescription<T>::value,
                  "Class 'setDescription' function has incorrect signature");
    static_assert(has_member_func_signature_print<T>::value, "Class 'print' function has incorrect signature");
  }
  ~InstructionInner() override = default;
  InstructionInner(const InstructionInner&) = delete;
  InstructionInner(InstructionInner&&) = delete;
  InstructionInner& operator=(const InstructionInner&) = delete;
  InstructionInner& operator=(InstructionInner&&) = delete;

  // Constructors from T (copy and move variants).
  explicit InstructionInner(T instruction) : instruction_(std::move(instruction))
  {
    static_assert(has_member_getDescription<T>::value, "Class does not have member function 'getDescription'");
    static_assert(has_member_setDescription<T>::value, "Class does not have member function 'setDescription'");
    static_assert(has_member_print<T>::value, "Class does not have member function 'print'");
    static_assert(has_member_func_signature_getDescription<T>::value,
                  "Class 'getDescription' function has incorrect signature");
    static_assert(has_member_func_signature_setDescription<T>::value,
                  "Class 'setDescription' function has incorrect signature");
    static_assert(has_member_func_signature_print<T>::value, "Class 'print' function has incorrect signature");
  }

  explicit InstructionInner(T&& instruction) : instruction_(std::move(instruction))
  {
    static_assert(has_member_getDescription<T>::value, "Class does not have member function 'getDescription'");
    static_assert(has_member_setDescription<T>::value, "Class does not have member function 'setDescription'");
    static_assert(has_member_print<T>::value, "Class does not have member function 'print'");
    static_assert(has_member_func_signature_getDescription<T>::value,
                  "Class 'getDescription' function has incorrect signature");
    static_assert(has_member_func_signature_setDescription<T>::value,
                  "Class 'setDescription' function has incorrect signature");
    static_assert(has_member_func_signature_print<T>::value, "Class 'print' function has incorrect signature");
  }

  std::unique_ptr<InstructionInnerBase> clone() const final { return std::make_unique<InstructionInner>(instruction_); }

  void* recover() final { return &instruction_; }

  const void* recover() const final { return &instruction_; }

  std::type_index getType() const final { return std::type_index(typeid(T)); }

  const std::string& getDescription() const final { return instruction_.getDescription(); }

  void setDescription(const std::string& description) final { instruction_.setDescription(description); }

  void print(const std::string& prefix) const final { instruction_.print(prefix); }

  bool operator==(const InstructionInnerBase& rhs) const final
  {
    // Compare class types before casting the incoming object to the T type
    if (rhs.getType() == getType())
    {
      auto instruction = static_cast<const T*>(rhs.recover());
      return instruction_ == *instruction;
    }
    return false;
  }

  bool operator!=(const InstructionInnerBase& rhs) const final { return !operator==(rhs); }

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
  {
    // If this line is removed a exception is thrown for unregistered cast need to too look into this.
    ar& boost::serialization::make_nvp("base", boost::serialization::base_object<InstructionInnerBase>(*this));
    ar& boost::serialization::make_nvp("impl", instruction_);
  }

  T instruction_;
};

}  // namespace detail_instruction
#endif  // SWIG
}  // namespace tesseract_planning

namespace boost
{
// Taken from pagmo to address the same issue
// NOTE: in some earlier versions of Boost (i.e., at least up to 1.67)
// the is_virtual_base_of type trait, used by the Boost serialization library, fails
// with a compile time error if a class is declared final. Thus, we provide a specialised
// implementation of this type trait to work around the issue. See:
// https://www.boost.org/doc/libs/1_52_0/libs/type_traits/doc/html/boost_typetraits/reference/is_virtual_base_of.html
// https://stackoverflow.com/questions/18982064/boost-serialization-of-base-class-of-final-subclass-error
// We never use virtual inheritance, thus the specialisation is always false.
template <typename T>
struct is_virtual_base_of<tesseract_planning::detail_instruction::InstructionInnerBase,
                          tesseract_planning::detail_instruction::InstructionInner<T>> : false_type
{
};
}  // namespace boost

namespace tesseract_planning
{
class Instruction
{
  template <typename T>
  using uncvref_t = std::remove_cv_t<typename std::remove_reference<T>::type>;

  // Enable the generic ctor only if ``T`` is not a ForwardKinematics (after removing const/reference qualifiers)
  // If ``T`` is of type ForwardKinematics we disable so it will use the copy or move constructors of this class.
  template <typename T>
  using generic_ctor_enabler = std::enable_if_t<!std::is_same<Instruction, uncvref_t<T>>::value, int>;

public:
  template <typename T, generic_ctor_enabler<T> = 0>
  Instruction(T&& instruction)  // NOLINT
    : instruction_(std::make_unique<detail_instruction::InstructionInner<uncvref_t<T>>>(instruction))
  {
  }

  // Destructor
  ~Instruction() = default;

  // Copy constructor
  Instruction(const Instruction& other) { instruction_ = other.instruction_->clone(); }

  // Move ctor.
  Instruction(Instruction&& other) noexcept { instruction_.swap(other.instruction_); }
  // Move assignment.
  Instruction& operator=(Instruction&& other) noexcept
  {
    instruction_.swap(other.instruction_);
    return (*this);
  }

  // Copy assignment.
  Instruction& operator=(const Instruction& other)
  {
    (*this) = Instruction(other);
    return (*this);
  }

  template <typename T, generic_ctor_enabler<T> = 0>
  Instruction& operator=(T&& other)
  {
    (*this) = Instruction(std::forward<T>(other));
    return (*this);
  }

  std::type_index getType() const { return instruction_->getType(); }

  const std::string& getDescription() const { return instruction_->getDescription(); }

  void setDescription(const std::string& description) { instruction_->setDescription(description); }

  void print(const std::string& prefix = "") const { instruction_->print(prefix); }

  bool operator==(const Instruction& rhs) const { return instruction_->operator==(*rhs.instruction_); }

  bool operator!=(const Instruction& rhs) const { return !operator==(rhs); }

  template <typename T>
  T& as()
  {
    if (getType() != typeid(T))
      throw std::bad_cast();

    auto p = static_cast<uncvref_t<T>*>(instruction_->recover());
    return *p;
  }

  template <typename T>
  const T& as() const
  {
    if (getType() != typeid(T))
      throw std::bad_cast();

    auto p = static_cast<const uncvref_t<T>*>(instruction_->recover());
    return *p;
  }

private:
  friend class boost::serialization::access;
  friend struct tesseract_planning::Serialization;

  Instruction()  // NOLINT
    : instruction_(nullptr)
  {
  }

  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
  {
    ar& boost::serialization::make_nvp("instruction", instruction_);
  }

  std::unique_ptr<detail_instruction::InstructionInnerBase> instruction_;
};

}  // namespace tesseract_planning

#ifdef SWIG
%template(Instructions) std::vector<tesseract_planning::Instruction>;
#else
BOOST_CLASS_TRACKING(tesseract_planning::Instruction, boost::serialization::track_never);
#endif  // SWIG

#endif  // TESSERACT_COMMAND_LANGUAGE_INSTRUCTION_H
