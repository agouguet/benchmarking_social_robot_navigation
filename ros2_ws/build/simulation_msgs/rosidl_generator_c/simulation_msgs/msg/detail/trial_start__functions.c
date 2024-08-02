// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from simulation_msgs:msg/TrialStart.idl
// generated code does not contain a copyright notice
#include "simulation_msgs/msg/detail/trial_start__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `trial_name`
#include "rosidl_runtime_c/string_functions.h"
// Member `spawn`
// Member `target`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `people`
#include "geometry_msgs/msg/detail/pose_array__functions.h"

bool
simulation_msgs__msg__TrialStart__init(simulation_msgs__msg__TrialStart * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    simulation_msgs__msg__TrialStart__fini(msg);
    return false;
  }
  // trial_name
  if (!rosidl_runtime_c__String__init(&msg->trial_name)) {
    simulation_msgs__msg__TrialStart__fini(msg);
    return false;
  }
  // trial_number
  // spawn
  if (!geometry_msgs__msg__Pose__init(&msg->spawn)) {
    simulation_msgs__msg__TrialStart__fini(msg);
    return false;
  }
  // target
  if (!geometry_msgs__msg__Pose__init(&msg->target)) {
    simulation_msgs__msg__TrialStart__fini(msg);
    return false;
  }
  // people
  if (!geometry_msgs__msg__PoseArray__init(&msg->people)) {
    simulation_msgs__msg__TrialStart__fini(msg);
    return false;
  }
  // time_limit
  return true;
}

void
simulation_msgs__msg__TrialStart__fini(simulation_msgs__msg__TrialStart * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // trial_name
  rosidl_runtime_c__String__fini(&msg->trial_name);
  // trial_number
  // spawn
  geometry_msgs__msg__Pose__fini(&msg->spawn);
  // target
  geometry_msgs__msg__Pose__fini(&msg->target);
  // people
  geometry_msgs__msg__PoseArray__fini(&msg->people);
  // time_limit
}

bool
simulation_msgs__msg__TrialStart__are_equal(const simulation_msgs__msg__TrialStart * lhs, const simulation_msgs__msg__TrialStart * rhs)
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
  // trial_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->trial_name), &(rhs->trial_name)))
  {
    return false;
  }
  // trial_number
  if (lhs->trial_number != rhs->trial_number) {
    return false;
  }
  // spawn
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->spawn), &(rhs->spawn)))
  {
    return false;
  }
  // target
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->target), &(rhs->target)))
  {
    return false;
  }
  // people
  if (!geometry_msgs__msg__PoseArray__are_equal(
      &(lhs->people), &(rhs->people)))
  {
    return false;
  }
  // time_limit
  if (lhs->time_limit != rhs->time_limit) {
    return false;
  }
  return true;
}

bool
simulation_msgs__msg__TrialStart__copy(
  const simulation_msgs__msg__TrialStart * input,
  simulation_msgs__msg__TrialStart * output)
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
  // trial_name
  if (!rosidl_runtime_c__String__copy(
      &(input->trial_name), &(output->trial_name)))
  {
    return false;
  }
  // trial_number
  output->trial_number = input->trial_number;
  // spawn
  if (!geometry_msgs__msg__Pose__copy(
      &(input->spawn), &(output->spawn)))
  {
    return false;
  }
  // target
  if (!geometry_msgs__msg__Pose__copy(
      &(input->target), &(output->target)))
  {
    return false;
  }
  // people
  if (!geometry_msgs__msg__PoseArray__copy(
      &(input->people), &(output->people)))
  {
    return false;
  }
  // time_limit
  output->time_limit = input->time_limit;
  return true;
}

simulation_msgs__msg__TrialStart *
simulation_msgs__msg__TrialStart__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  simulation_msgs__msg__TrialStart * msg = (simulation_msgs__msg__TrialStart *)allocator.allocate(sizeof(simulation_msgs__msg__TrialStart), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(simulation_msgs__msg__TrialStart));
  bool success = simulation_msgs__msg__TrialStart__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
simulation_msgs__msg__TrialStart__destroy(simulation_msgs__msg__TrialStart * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    simulation_msgs__msg__TrialStart__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
simulation_msgs__msg__TrialStart__Sequence__init(simulation_msgs__msg__TrialStart__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  simulation_msgs__msg__TrialStart * data = NULL;

  if (size) {
    data = (simulation_msgs__msg__TrialStart *)allocator.zero_allocate(size, sizeof(simulation_msgs__msg__TrialStart), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = simulation_msgs__msg__TrialStart__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        simulation_msgs__msg__TrialStart__fini(&data[i - 1]);
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
simulation_msgs__msg__TrialStart__Sequence__fini(simulation_msgs__msg__TrialStart__Sequence * array)
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
      simulation_msgs__msg__TrialStart__fini(&array->data[i]);
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

simulation_msgs__msg__TrialStart__Sequence *
simulation_msgs__msg__TrialStart__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  simulation_msgs__msg__TrialStart__Sequence * array = (simulation_msgs__msg__TrialStart__Sequence *)allocator.allocate(sizeof(simulation_msgs__msg__TrialStart__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = simulation_msgs__msg__TrialStart__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
simulation_msgs__msg__TrialStart__Sequence__destroy(simulation_msgs__msg__TrialStart__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    simulation_msgs__msg__TrialStart__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
simulation_msgs__msg__TrialStart__Sequence__are_equal(const simulation_msgs__msg__TrialStart__Sequence * lhs, const simulation_msgs__msg__TrialStart__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!simulation_msgs__msg__TrialStart__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
simulation_msgs__msg__TrialStart__Sequence__copy(
  const simulation_msgs__msg__TrialStart__Sequence * input,
  simulation_msgs__msg__TrialStart__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(simulation_msgs__msg__TrialStart);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    simulation_msgs__msg__TrialStart * data =
      (simulation_msgs__msg__TrialStart *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!simulation_msgs__msg__TrialStart__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          simulation_msgs__msg__TrialStart__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!simulation_msgs__msg__TrialStart__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
