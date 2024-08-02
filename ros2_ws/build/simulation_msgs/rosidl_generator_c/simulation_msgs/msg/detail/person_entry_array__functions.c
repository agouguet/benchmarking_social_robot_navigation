// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from simulation_msgs:msg/PersonEntryArray.idl
// generated code does not contain a copyright notice
#include "simulation_msgs/msg/detail/person_entry_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `people`
#include "simulation_msgs/msg/detail/person_entry__functions.h"

bool
simulation_msgs__msg__PersonEntryArray__init(simulation_msgs__msg__PersonEntryArray * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    simulation_msgs__msg__PersonEntryArray__fini(msg);
    return false;
  }
  // people
  if (!simulation_msgs__msg__PersonEntry__Sequence__init(&msg->people, 0)) {
    simulation_msgs__msg__PersonEntryArray__fini(msg);
    return false;
  }
  return true;
}

void
simulation_msgs__msg__PersonEntryArray__fini(simulation_msgs__msg__PersonEntryArray * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // people
  simulation_msgs__msg__PersonEntry__Sequence__fini(&msg->people);
}

bool
simulation_msgs__msg__PersonEntryArray__are_equal(const simulation_msgs__msg__PersonEntryArray * lhs, const simulation_msgs__msg__PersonEntryArray * rhs)
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
  // people
  if (!simulation_msgs__msg__PersonEntry__Sequence__are_equal(
      &(lhs->people), &(rhs->people)))
  {
    return false;
  }
  return true;
}

bool
simulation_msgs__msg__PersonEntryArray__copy(
  const simulation_msgs__msg__PersonEntryArray * input,
  simulation_msgs__msg__PersonEntryArray * output)
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
  // people
  if (!simulation_msgs__msg__PersonEntry__Sequence__copy(
      &(input->people), &(output->people)))
  {
    return false;
  }
  return true;
}

simulation_msgs__msg__PersonEntryArray *
simulation_msgs__msg__PersonEntryArray__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  simulation_msgs__msg__PersonEntryArray * msg = (simulation_msgs__msg__PersonEntryArray *)allocator.allocate(sizeof(simulation_msgs__msg__PersonEntryArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(simulation_msgs__msg__PersonEntryArray));
  bool success = simulation_msgs__msg__PersonEntryArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
simulation_msgs__msg__PersonEntryArray__destroy(simulation_msgs__msg__PersonEntryArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    simulation_msgs__msg__PersonEntryArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
simulation_msgs__msg__PersonEntryArray__Sequence__init(simulation_msgs__msg__PersonEntryArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  simulation_msgs__msg__PersonEntryArray * data = NULL;

  if (size) {
    data = (simulation_msgs__msg__PersonEntryArray *)allocator.zero_allocate(size, sizeof(simulation_msgs__msg__PersonEntryArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = simulation_msgs__msg__PersonEntryArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        simulation_msgs__msg__PersonEntryArray__fini(&data[i - 1]);
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
simulation_msgs__msg__PersonEntryArray__Sequence__fini(simulation_msgs__msg__PersonEntryArray__Sequence * array)
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
      simulation_msgs__msg__PersonEntryArray__fini(&array->data[i]);
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

simulation_msgs__msg__PersonEntryArray__Sequence *
simulation_msgs__msg__PersonEntryArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  simulation_msgs__msg__PersonEntryArray__Sequence * array = (simulation_msgs__msg__PersonEntryArray__Sequence *)allocator.allocate(sizeof(simulation_msgs__msg__PersonEntryArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = simulation_msgs__msg__PersonEntryArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
simulation_msgs__msg__PersonEntryArray__Sequence__destroy(simulation_msgs__msg__PersonEntryArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    simulation_msgs__msg__PersonEntryArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
simulation_msgs__msg__PersonEntryArray__Sequence__are_equal(const simulation_msgs__msg__PersonEntryArray__Sequence * lhs, const simulation_msgs__msg__PersonEntryArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!simulation_msgs__msg__PersonEntryArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
simulation_msgs__msg__PersonEntryArray__Sequence__copy(
  const simulation_msgs__msg__PersonEntryArray__Sequence * input,
  simulation_msgs__msg__PersonEntryArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(simulation_msgs__msg__PersonEntryArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    simulation_msgs__msg__PersonEntryArray * data =
      (simulation_msgs__msg__PersonEntryArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!simulation_msgs__msg__PersonEntryArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          simulation_msgs__msg__PersonEntryArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!simulation_msgs__msg__PersonEntryArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
