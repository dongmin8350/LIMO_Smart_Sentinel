// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from instant_messaging_interfaces:msg/Options.idl
// generated code does not contain a copyright notice

#ifndef INSTANT_MESSAGING_INTERFACES__MSG__DETAIL__OPTIONS__STRUCT_HPP_
#define INSTANT_MESSAGING_INTERFACES__MSG__DETAIL__OPTIONS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__instant_messaging_interfaces__msg__Options __attribute__((deprecated))
#else
# define DEPRECATED__instant_messaging_interfaces__msg__Options __declspec(deprecated)
#endif

namespace instant_messaging_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Options_
{
  using Type = Options_<ContainerAllocator>;

  explicit Options_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->question = "";
    }
  }

  explicit Options_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : question(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->question = "";
    }
  }

  // field types and members
  using _question_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _question_type question;
  using _options_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _options_type options;

  // setters for named parameter idiom
  Type & set__question(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->question = _arg;
    return *this;
  }
  Type & set__options(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->options = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    instant_messaging_interfaces::msg::Options_<ContainerAllocator> *;
  using ConstRawPtr =
    const instant_messaging_interfaces::msg::Options_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<instant_messaging_interfaces::msg::Options_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<instant_messaging_interfaces::msg::Options_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      instant_messaging_interfaces::msg::Options_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<instant_messaging_interfaces::msg::Options_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      instant_messaging_interfaces::msg::Options_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<instant_messaging_interfaces::msg::Options_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<instant_messaging_interfaces::msg::Options_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<instant_messaging_interfaces::msg::Options_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__instant_messaging_interfaces__msg__Options
    std::shared_ptr<instant_messaging_interfaces::msg::Options_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__instant_messaging_interfaces__msg__Options
    std::shared_ptr<instant_messaging_interfaces::msg::Options_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Options_ & other) const
  {
    if (this->question != other.question) {
      return false;
    }
    if (this->options != other.options) {
      return false;
    }
    return true;
  }
  bool operator!=(const Options_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Options_

// alias to use template instance with default allocator
using Options =
  instant_messaging_interfaces::msg::Options_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace instant_messaging_interfaces

#endif  // INSTANT_MESSAGING_INTERFACES__MSG__DETAIL__OPTIONS__STRUCT_HPP_
