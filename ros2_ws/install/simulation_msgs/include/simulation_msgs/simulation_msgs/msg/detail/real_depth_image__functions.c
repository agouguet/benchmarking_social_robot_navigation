// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from simulation_msgs:msg/RealDepthImage.idl
// generated code does not contain a copyright notice
#include "simulation_msgs/msg/detail/real_depth_image__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
simulation_msgs__msg__RealDepthImage__init(simulation_msgs__msg__RealDepthImage * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    simulation_msgs__msg__RealDepthImage__fini(msg);
    return false;
  }
  // data
  if (!rosidl_runtime_c__float__Sequence__init(&msg->data, 0)) {
    simulation_msgs__msg__RealDepthImage__fini(msg);
    return false;
  }
  return true;
}

void
simulation_msgs__msg__RealDepthImage__fini(simulation_msgs__msg__RealDepthImage * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // data
  rosidl_runtime_c__float__Sequence__fini(&msg->data);
}

bool
simulation_msgs__msg__RealDepthImage__are_equal(const simulation_msgs__msg__RealDepthImage * lhs, const simulation_msgs__msg__RealDepthImage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // data
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->data), &(rhs->data)))
  {
    return false;
  }
  return true;
}

bool
simulation_msgs__msg__RealDepthImage__copy(
  const simulation_msgs__msg__RealDepthImage * input,
  simulation_msgs__msg__RealDepthImage * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // data
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->data), &(output->data)))
  {
    return false;
  }
  return true;
}

simulation_msgs__msg__RealDepthImage *
simulation_msgs__msg__RealDepthImage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  simulation_msgs__msg__RealDepthImage * msg = (simulation_msgs__msg__RealDepthImage *)allocator.allocate(sizeof(simulation_msgs__msg__RealDepthImage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(simulation_msgs__msg__RealDepthImage));
  bool success = simulation_msgs__msg__RealDepthImage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
simulation_msgs__msg__RealDepthImage__destroy(simulation_msgs__msg__RealDepthImage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    simulation_msgs__msg__RealDepthImage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
simulation_msgs__msg__RealDepthImage__Sequence__init(simulation_msgs__msg__RealDepthImage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  simulation_msgs__msg__RealDepthImage * data = NULL;

  if (size) {
    data = (simulation_msgs__msg__RealDepthImage *)allocator.zero_allocate(size, sizeof(simulation_msgs__msg__RealDepthImage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = simulation_msgs__msg__RealDepthImage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        simulation_msgs__msg__RealDepthImage__fini(&data[i - 1]);
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
simulation_msgs__msg__RealDepthImage__Sequence__fini(simulation_msgs__msg__RealDepthImage__Sequence * array)
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
      simulation_msgs__msg__RealDepthImage__fini(&array->data[i]);
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

simulation_msgs__msg__RealDepthImage__Sequence *
simulation_msgs__msg__RealDepthImage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  simulation_msgs__msg__RealDepthImage__Sequence * array = (simulation_msgs__msg__RealDepthImage__Sequence *)allocator.allocate(sizeof(simulation_msgs__msg__RealDepthImage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = simulation_msgs__msg__RealDepthImage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
simulation_msgs__msg__RealDepthImage__Sequence__destroy(simulation_msgs__msg__RealDepthImage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    simulation_msgs__msg__RealDepthImage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
simulation_msgs__msg__RealDepthImage__Sequence__are_equal(const simulation_msgs__msg__RealDepthImage__Sequence * lhs, const simulation_msgs__msg__RealDepthImage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!simulation_msgs__msg__RealDepthImage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
simulation_msgs__msg__RealDepthImage__Sequence__copy(
  const simulation_msgs__msg__RealDepthImage__Sequence * input,
  simulation_msgs__msg__RealDepthImage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(simulation_msgs__msg__RealDepthImage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    simulation_msgs__msg__RealDepthImage * data =
      (simulation_msgs__msg__RealDepthImage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!simulation_msgs__msg__RealDepthImage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          simulation_msgs__msg__RealDepthImage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!simulation_msgs__msg__RealDepthImage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
