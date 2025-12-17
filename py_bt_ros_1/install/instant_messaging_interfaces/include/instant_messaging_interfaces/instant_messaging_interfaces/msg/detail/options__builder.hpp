// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from instant_messaging_interfaces:msg/Options.idl
// generated code does not contain a copyright notice

#ifndef INSTANT_MESSAGING_INTERFACES__MSG__DETAIL__OPTIONS__BUILDER_HPP_
#define INSTANT_MESSAGING_INTERFACES__MSG__DETAIL__OPTIONS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "instant_messaging_interfaces/msg/detail/options__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace instant_messaging_interfaces
{

namespace msg
{

namespace builder
{

class Init_Options_options
{
public:
  explicit Init_Options_options(::instant_messaging_interfaces::msg::Options & msg)
  : msg_(msg)
  {}
  ::instant_messaging_interfaces::msg::Options options(::instant_messaging_interfaces::msg::Options::_options_type arg)
  {
    msg_.options = std::move(arg);
    return std::move(msg_);
  }

private:
  ::instant_messaging_interfaces::msg::Options msg_;
};

class Init_Options_question
{
public:
  Init_Options_question()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Options_options question(::instant_messaging_interfaces::msg::Options::_question_type arg)
  {
    msg_.question = std::move(arg);
    return Init_Options_options(msg_);
  }

private:
  ::instant_messaging_interfaces::msg::Options msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::instant_messaging_interfaces::msg::Options>()
{
  return instant_messaging_interfaces::msg::builder::Init_Options_question();
}

}  // namespace instant_messaging_interfaces

#endif  // INSTANT_MESSAGING_INTERFACES__MSG__DETAIL__OPTIONS__BUILDER_HPP_
