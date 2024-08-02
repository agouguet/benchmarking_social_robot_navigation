// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from simulation_msgs:msg/PersonEntry.idl
// generated code does not contain a copyright notice
#include "simulation_msgs/msg/detail/person_entry__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `twist`
#include "geometry_msgs/msg/detail/twist__functions.h"

bool
simulation_msgs__msg__PersonEntry__init(simulation_msgs__msg__PersonEntry * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    simulation_msgs__msg__PersonEntry__fini(msg);
    return false;
  }
  // track_id
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    simulation_msgs__msg__PersonEntry__fini(msg);
    return false;
  }
  // twist
  if (!geometry_msgs__msg__Twist__init(&msg->twist)) {
    simulation_msgs__msg__PersonEntry__fini(msg);
    return false;
  }
  return true;
}

void
simulation_msgs__msg__PersonEntry__fini(simulation_msgs__msg__PersonEntry * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // track_id
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
  // twist
  geometry_msgs__msg__Twist__fini(&msg->twist);
}

bool
simulation_msgs__msg__PersonEntry__are_equal(const simulation_msgs__msg__PersonEntry * lhs, const simulation_msgs__msg__PersonEntry * rhs)
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
  // track_id
  if (lhs->track_id != rhs->track_id) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // twist
  if (!geometry_msgs__msg__Twist__are_equal(
      &(lhs->twist), &(rhs->twist)))
  {
    return false;
  }
  return true;
}

bool
simulation_msgs__msg__PersonEntry__copy(
  const simulation_msgs__msg__PersonEntry * input,
  simulation_msgs__msg__PersonEntry * output)
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
  // track_id
  output->track_id = input->track_id;
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // twist
  if (!geometry_msgs__msg__Twist__copy(
      &(input->twist), &(output->twist)))
  {
    return false;
  }
  return true;
}

simulation_msgs__msg__PersonEntry *
simulation_msgs__msg__PersonEntry__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  simulation_msgs__msg__PersonEntry * msg = (simulation_msgs__msg__PersonEntry *)allocator.allocate(sizeof(simulation_msgs__msg__PersonEntry), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(simulation_msgs__msg__PersonEntry));
  bool success = simulation_msgs__msg__PersonEntry__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
simulation_msgs__msg__PersonEntry__destroy(simulation_msgs__msg__PersonEntry * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    simulation_msgs__msg__PersonEntry__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
simulation_msgs__msg__PersonEntry__Sequence__init(simulation_msgs__msg__PersonEntry__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  simulation_msgs__msg__PersonEntry * data = NULL;

  if (size) {
    data = (simulation_msgs__msg__PersonEntry *)allocator.zero_allocate(size, sizeof(simulation_msgs__msg__PersonEntry), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = simulation_msgs__msg__PersonEntry__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        simulation_msgs__msg__PersonEntry__fini(&data[i - 1]);
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
simulation_msgs__msg__PersonEntry__Sequence__fini(simulation_msgs__msg__PersonEntry__Sequence * array)
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
      simulation_msgs__msg__PersonEntry__fini(&array->data[i]);
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

simulation_msgs__msg__PersonEntry__Sequence *
simulation_msgs__msg__PersonEntry__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  simulation_msgs__msg__PersonEntry__Sequence * array = (simulation_msgs__msg__PersonEntry__Sequence *)allocator.allocate(sizeof(simulation_msgs__msg__PersonEntry__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = simulation_msgs__msg__PersonEntry__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
simulation_msgs__msg__PersonEntry__Sequence__destroy(simulation_msgs__msg__PersonEntry__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    simulation_msgs__msg__PersonEntry__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
simulation_msgs__msg__PersonEntry__Sequence__are_equal(const simulation_msgs__msg__PersonEntry__Sequence * lhs, const simulation_msgs__msg__PersonEntry__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!simulation_msgs__msg__PersonEntry__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
simulation_msgs__msg__PersonEntry__Sequence__copy(
  const simulation_msgs__msg__PersonEntry__Sequence * input,
  simulation_msgs__msg__PersonEntry__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(simulation_msgs__msg__PersonEntry);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    simulation_msgs__msg__PersonEntry * data =
      (simulation_msgs__msg__PersonEntry *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!simulation_msgs__msg__PersonEntry__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          simulation_msgs__msg__PersonEntry__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!simulation_msgs__msg__PersonEntry__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
