// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from instant_messaging_interfaces:msg/Options.idl
// generated code does not contain a copyright notice

#ifndef INSTANT_MESSAGING_INTERFACES__MSG__DETAIL__OPTIONS__TRAITS_HPP_
#define INSTANT_MESSAGING_INTERFACES__MSG__DETAIL__OPTIONS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "instant_messaging_interfaces/msg/detail/options__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace instant_messaging_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Options & msg,
  std::ostream & out)
{
  out << "{";
  // member: question
  {
    out << "question: ";
    rosidl_generator_traits::value_to_yaml(msg.question, out);
    out << ", ";
  }

  // member: options
  {
    if (msg.options.size() == 0) {
      out << "options: []";
    } else {
      out << "options: [";
      size_t pending_items = msg.options.size();
      for (auto item : msg.options) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Options & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: question
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "question: ";
    rosidl_generator_traits::value_to_yaml(msg.question, out);
    out << "\n";
  }

  // member: options
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.options.size() == 0) {
      out << "options: []\n";
    } else {
      out << "options:\n";
      for (auto item : msg.options) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Options & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace instant_messaging_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use instant_messaging_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const instant_messaging_interfaces::msg::Options & msg,
  std::ostream & out, size_t indentation = 0)
{
  instant_messaging_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use instant_messaging_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const instant_messaging_interfaces::msg::Options & msg)
{
  return instant_messaging_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<instant_messaging_interfaces::msg::Options>()
{
  return "instant_messaging_interfaces::msg::Options";
}

template<>
inline const char * name<instant_messaging_interfaces::msg::Options>()
{
  return "instant_messaging_interfaces/msg/Options";
}

template<>
struct has_fixed_size<instant_messaging_interfaces::msg::Options>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<instant_messaging_interfaces::msg::Options>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<instant_messaging_interfaces::msg::Options>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INSTANT_MESSAGING_INTERFACES__MSG__DETAIL__OPTIONS__TRAITS_HPP_
