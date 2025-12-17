// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from instant_messaging_interfaces:msg/Options.idl
// generated code does not contain a copyright notice
#include "instant_messaging_interfaces/msg/detail/options__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "instant_messaging_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "instant_messaging_interfaces/msg/detail/options__struct.h"
#include "instant_messaging_interfaces/msg/detail/options__functions.h"
#include "fastcdr/Cdr.h"

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

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // options, question
#include "rosidl_runtime_c/string_functions.h"  // options, question

// forward declare type support functions


using _Options__ros_msg_type = instant_messaging_interfaces__msg__Options;

static bool _Options__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Options__ros_msg_type * ros_message = static_cast<const _Options__ros_msg_type *>(untyped_ros_message);
  // Field name: question
  {
    const rosidl_runtime_c__String * str = &ros_message->question;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: options
  {
    size_t size = ros_message->options.size;
    auto array_ptr = ros_message->options.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      const rosidl_runtime_c__String * str = &array_ptr[i];
      if (str->capacity == 0 || str->capacity <= str->size) {
        fprintf(stderr, "string capacity not greater than size\n");
        return false;
      }
      if (str->data[str->size] != '\0') {
        fprintf(stderr, "string not null-terminated\n");
        return false;
      }
      cdr << str->data;
    }
  }

  return true;
}

static bool _Options__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Options__ros_msg_type * ros_message = static_cast<_Options__ros_msg_type *>(untyped_ros_message);
  // Field name: question
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->question.data) {
      rosidl_runtime_c__String__init(&ros_message->question);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->question,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'question'\n");
      return false;
    }
  }

  // Field name: options
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.getState();
    bool correct_size = cdr.jump(size);
    cdr.setState(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->options.data) {
      rosidl_runtime_c__String__Sequence__fini(&ros_message->options);
    }
    if (!rosidl_runtime_c__String__Sequence__init(&ros_message->options, size)) {
      fprintf(stderr, "failed to create array for field 'options'");
      return false;
    }
    auto array_ptr = ros_message->options.data;
    for (size_t i = 0; i < size; ++i) {
      std::string tmp;
      cdr >> tmp;
      auto & ros_i = array_ptr[i];
      if (!ros_i.data) {
        rosidl_runtime_c__String__init(&ros_i);
      }
      bool succeeded = rosidl_runtime_c__String__assign(
        &ros_i,
        tmp.c_str());
      if (!succeeded) {
        fprintf(stderr, "failed to assign string into field 'options'\n");
        return false;
      }
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_instant_messaging_interfaces
size_t get_serialized_size_instant_messaging_interfaces__msg__Options(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Options__ros_msg_type * ros_message = static_cast<const _Options__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name question
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->question.size + 1);
  // field.name options
  {
    size_t array_size = ros_message->options.size;
    auto array_ptr = ros_message->options.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (array_ptr[index].size + 1);
    }
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Options__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_instant_messaging_interfaces__msg__Options(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_instant_messaging_interfaces
size_t max_serialized_size_instant_messaging_interfaces__msg__Options(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: question
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: options
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = instant_messaging_interfaces__msg__Options;
    is_plain =
      (
      offsetof(DataType, options) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _Options__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_instant_messaging_interfaces__msg__Options(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Options = {
  "instant_messaging_interfaces::msg",
  "Options",
  _Options__cdr_serialize,
  _Options__cdr_deserialize,
  _Options__get_serialized_size,
  _Options__max_serialized_size
};

static rosidl_message_type_support_t _Options__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Options,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, instant_messaging_interfaces, msg, Options)() {
  return &_Options__type_support;
}

#if defined(__cplusplus)
}
#endif
