// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from agents_msgs:msg/AgentTrajectory.idl
// generated code does not contain a copyright notice
#include "agents_msgs/msg/detail/agent_trajectory__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `poses`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
agents_msgs__msg__AgentTrajectory__init(agents_msgs__msg__AgentTrajectory * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    agents_msgs__msg__AgentTrajectory__fini(msg);
    return false;
  }
  // id
  // poses
  if (!geometry_msgs__msg__Point__Sequence__init(&msg->poses, 0)) {
    agents_msgs__msg__AgentTrajectory__fini(msg);
    return false;
  }
  return true;
}

void
agents_msgs__msg__AgentTrajectory__fini(agents_msgs__msg__AgentTrajectory * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // id
  // poses
  geometry_msgs__msg__Point__Sequence__fini(&msg->poses);
}

bool
agents_msgs__msg__AgentTrajectory__are_equal(const agents_msgs__msg__AgentTrajectory * lhs, const agents_msgs__msg__AgentTrajectory * rhs)
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
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // poses
  if (!geometry_msgs__msg__Point__Sequence__are_equal(
      &(lhs->poses), &(rhs->poses)))
  {
    return false;
  }
  return true;
}

bool
agents_msgs__msg__AgentTrajectory__copy(
  const agents_msgs__msg__AgentTrajectory * input,
  agents_msgs__msg__AgentTrajectory * output)
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
  // id
  output->id = input->id;
  // poses
  if (!geometry_msgs__msg__Point__Sequence__copy(
      &(input->poses), &(output->poses)))
  {
    return false;
  }
  return true;
}

agents_msgs__msg__AgentTrajectory *
agents_msgs__msg__AgentTrajectory__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  agents_msgs__msg__AgentTrajectory * msg = (agents_msgs__msg__AgentTrajectory *)allocator.allocate(sizeof(agents_msgs__msg__AgentTrajectory), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(agents_msgs__msg__AgentTrajectory));
  bool success = agents_msgs__msg__AgentTrajectory__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
agents_msgs__msg__AgentTrajectory__destroy(agents_msgs__msg__AgentTrajectory * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    agents_msgs__msg__AgentTrajectory__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
agents_msgs__msg__AgentTrajectory__Sequence__init(agents_msgs__msg__AgentTrajectory__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  agents_msgs__msg__AgentTrajectory * data = NULL;

  if (size) {
    data = (agents_msgs__msg__AgentTrajectory *)allocator.zero_allocate(size, sizeof(agents_msgs__msg__AgentTrajectory), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = agents_msgs__msg__AgentTrajectory__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        agents_msgs__msg__AgentTrajectory__fini(&data[i - 1]);
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
agents_msgs__msg__AgentTrajectory__Sequence__fini(agents_msgs__msg__AgentTrajectory__Sequence * array)
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
      agents_msgs__msg__AgentTrajectory__fini(&array->data[i]);
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

agents_msgs__msg__AgentTrajectory__Sequence *
agents_msgs__msg__AgentTrajectory__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  agents_msgs__msg__AgentTrajectory__Sequence * array = (agents_msgs__msg__AgentTrajectory__Sequence *)allocator.allocate(sizeof(agents_msgs__msg__AgentTrajectory__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = agents_msgs__msg__AgentTrajectory__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
agents_msgs__msg__AgentTrajectory__Sequence__destroy(agents_msgs__msg__AgentTrajectory__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    agents_msgs__msg__AgentTrajectory__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
agents_msgs__msg__AgentTrajectory__Sequence__are_equal(const agents_msgs__msg__AgentTrajectory__Sequence * lhs, const agents_msgs__msg__AgentTrajectory__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!agents_msgs__msg__AgentTrajectory__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
agents_msgs__msg__AgentTrajectory__Sequence__copy(
  const agents_msgs__msg__AgentTrajectory__Sequence * input,
  agents_msgs__msg__AgentTrajectory__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(agents_msgs__msg__AgentTrajectory);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    agents_msgs__msg__AgentTrajectory * data =
      (agents_msgs__msg__AgentTrajectory *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!agents_msgs__msg__AgentTrajectory__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          agents_msgs__msg__AgentTrajectory__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!agents_msgs__msg__AgentTrajectory__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
