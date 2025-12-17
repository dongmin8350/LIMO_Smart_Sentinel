// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from instant_messaging_interfaces:msg/Options.idl
// generated code does not contain a copyright notice

#ifndef INSTANT_MESSAGING_INTERFACES__MSG__DETAIL__OPTIONS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define INSTANT_MESSAGING_INTERFACES__MSG__DETAIL__OPTIONS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "instant_messaging_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "instant_messaging_interfaces/msg/detail/options__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace instant_messaging_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_instant_messaging_interfaces
cdr_serialize(
  const instant_messaging_interfaces::msg::Options & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_instant_messaging_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  instant_messaging_interfaces::msg::Options & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_instant_messaging_interfaces
get_serialized_size(
  const instant_messaging_interfaces::msg::Options & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_instant_messaging_interfaces
max_serialized_size_Options(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace instant_messaging_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_instant_messaging_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, instant_messaging_interfaces, msg, Options)();

#ifdef __cplusplus
}
#endif

#endif  // INSTANT_MESSAGING_INTERFACES__MSG__DETAIL__OPTIONS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
