// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from instant_messaging_interfaces:msg/Options.idl
// generated code does not contain a copyright notice

#ifndef INSTANT_MESSAGING_INTERFACES__MSG__DETAIL__OPTIONS__STRUCT_H_
#define INSTANT_MESSAGING_INTERFACES__MSG__DETAIL__OPTIONS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'question'
// Member 'options'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Options in the package instant_messaging_interfaces.
typedef struct instant_messaging_interfaces__msg__Options
{
  rosidl_runtime_c__String question;
  rosidl_runtime_c__String__Sequence options;
} instant_messaging_interfaces__msg__Options;

// Struct for a sequence of instant_messaging_interfaces__msg__Options.
typedef struct instant_messaging_interfaces__msg__Options__Sequence
{
  instant_messaging_interfaces__msg__Options * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} instant_messaging_interfaces__msg__Options__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INSTANT_MESSAGING_INTERFACES__MSG__DETAIL__OPTIONS__STRUCT_H_
