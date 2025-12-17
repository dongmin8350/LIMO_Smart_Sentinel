// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from instant_messaging_interfaces:msg/Options.idl
// generated code does not contain a copyright notice
#include "instant_messaging_interfaces/msg/detail/options__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `question`
// Member `options`
#include "rosidl_runtime_c/string_functions.h"

bool
instant_messaging_interfaces__msg__Options__init(instant_messaging_interfaces__msg__Options * msg)
{
  if (!msg) {
    return false;
  }
  // question
  if (!rosidl_runtime_c__String__init(&msg->question)) {
    instant_messaging_interfaces__msg__Options__fini(msg);
    return false;
  }
  // options
  if (!rosidl_runtime_c__String__Sequence__init(&msg->options, 0)) {
    instant_messaging_interfaces__msg__Options__fini(msg);
    return false;
  }
  return true;
}

void
instant_messaging_interfaces__msg__Options__fini(instant_messaging_interfaces__msg__Options * msg)
{
  if (!msg) {
    return;
  }
  // question
  rosidl_runtime_c__String__fini(&msg->question);
  // options
  rosidl_runtime_c__String__Sequence__fini(&msg->options);
}

bool
instant_messaging_interfaces__msg__Options__are_equal(const instant_messaging_interfaces__msg__Options * lhs, const instant_messaging_interfaces__msg__Options * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // question
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->question), &(rhs->question)))
  {
    return false;
  }
  // options
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->options), &(rhs->options)))
  {
    return false;
  }
  return true;
}

bool
instant_messaging_interfaces__msg__Options__copy(
  const instant_messaging_interfaces__msg__Options * input,
  instant_messaging_interfaces__msg__Options * output)
{
  if (!input || !output) {
    return false;
  }
  // question
  if (!rosidl_runtime_c__String__copy(
      &(input->question), &(output->question)))
  {
    return false;
  }
  // options
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->options), &(output->options)))
  {
    return false;
  }
  return true;
}

instant_messaging_interfaces__msg__Options *
instant_messaging_interfaces__msg__Options__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  instant_messaging_interfaces__msg__Options * msg = (instant_messaging_interfaces__msg__Options *)allocator.allocate(sizeof(instant_messaging_interfaces__msg__Options), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(instant_messaging_interfaces__msg__Options));
  bool success = instant_messaging_interfaces__msg__Options__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
instant_messaging_interfaces__msg__Options__destroy(instant_messaging_interfaces__msg__Options * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    instant_messaging_interfaces__msg__Options__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
instant_messaging_interfaces__msg__Options__Sequence__init(instant_messaging_interfaces__msg__Options__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  instant_messaging_interfaces__msg__Options * data = NULL;

  if (size) {
    data = (instant_messaging_interfaces__msg__Options *)allocator.zero_allocate(size, sizeof(instant_messaging_interfaces__msg__Options), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = instant_messaging_interfaces__msg__Options__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        instant_messaging_interfaces__msg__Options__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
instant_messaging_interfaces__msg__Options__Sequence__fini(instant_messaging_interfaces__msg__Options__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      instant_messaging_interfaces__msg__Options__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

instant_messaging_interfaces__msg__Options__Sequence *
instant_messaging_interfaces__msg__Options__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  instant_messaging_interfaces__msg__Options__Sequence * array = (instant_messaging_interfaces__msg__Options__Sequence *)allocator.allocate(sizeof(instant_messaging_interfaces__msg__Options__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = instant_messaging_interfaces__msg__Options__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
instant_messaging_interfaces__msg__Options__Sequence__destroy(instant_messaging_interfaces__msg__Options__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    instant_messaging_interfaces__msg__Options__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
instant_messaging_interfaces__msg__Options__Sequence__are_equal(const instant_messaging_interfaces__msg__Options__Sequence * lhs, const instant_messaging_interfaces__msg__Options__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!instant_messaging_interfaces__msg__Options__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
instant_messaging_interfaces__msg__Options__Sequence__copy(
  const instant_messaging_interfaces__msg__Options__Sequence * input,
  instant_messaging_interfaces__msg__Options__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(instant_messaging_interfaces__msg__Options);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    instant_messaging_interfaces__msg__Options * data =
      (instant_messaging_interfaces__msg__Options *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!instant_messaging_interfaces__msg__Options__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          instant_messaging_interfaces__msg__Options__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!instant_messaging_interfaces__msg__Options__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
