// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from simulation_msgs:msg/SceneInfo.idl
// generated code does not contain a copyright notice
#include "simulation_msgs/msg/detail/scene_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `scenario_name`
// Member `environment`
#include "rosidl_runtime_c/string_functions.h"
// Member `robot_start_pose`
// Member `robot_target_pose`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
simulation_msgs__msg__SceneInfo__init(simulation_msgs__msg__SceneInfo * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    simulation_msgs__msg__SceneInfo__fini(msg);
    return false;
  }
  // scenario_name
  if (!rosidl_runtime_c__String__init(&msg->scenario_name)) {
    simulation_msgs__msg__SceneInfo__fini(msg);
    return false;
  }
  // robot_start_pose
  if (!geometry_msgs__msg__Pose__init(&msg->robot_start_pose)) {
    simulation_msgs__msg__SceneInfo__fini(msg);
    return false;
  }
  // robot_target_pose
  if (!geometry_msgs__msg__Pose__init(&msg->robot_target_pose)) {
    simulation_msgs__msg__SceneInfo__fini(msg);
    return false;
  }
  // num_people
  // num_groups
  // environment
  if (!rosidl_runtime_c__String__init(&msg->environment)) {
    simulation_msgs__msg__SceneInfo__fini(msg);
    return false;
  }
  return true;
}

void
simulation_msgs__msg__SceneInfo__fini(simulation_msgs__msg__SceneInfo * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // scenario_name
  rosidl_runtime_c__String__fini(&msg->scenario_name);
  // robot_start_pose
  geometry_msgs__msg__Pose__fini(&msg->robot_start_pose);
  // robot_target_pose
  geometry_msgs__msg__Pose__fini(&msg->robot_target_pose);
  // num_people
  // num_groups
  // environment
  rosidl_runtime_c__String__fini(&msg->environment);
}

bool
simulation_msgs__msg__SceneInfo__are_equal(const simulation_msgs__msg__SceneInfo * lhs, const simulation_msgs__msg__SceneInfo * rhs)
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
  // scenario_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->scenario_name), &(rhs->scenario_name)))
  {
    return false;
  }
  // robot_start_pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->robot_start_pose), &(rhs->robot_start_pose)))
  {
    return false;
  }
  // robot_target_pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->robot_target_pose), &(rhs->robot_target_pose)))
  {
    return false;
  }
  // num_people
  if (lhs->num_people != rhs->num_people) {
    return false;
  }
  // num_groups
  if (lhs->num_groups != rhs->num_groups) {
    return false;
  }
  // environment
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->environment), &(rhs->environment)))
  {
    return false;
  }
  return true;
}

bool
simulation_msgs__msg__SceneInfo__copy(
  const simulation_msgs__msg__SceneInfo * input,
  simulation_msgs__msg__SceneInfo * output)
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
  // scenario_name
  if (!rosidl_runtime_c__String__copy(
      &(input->scenario_name), &(output->scenario_name)))
  {
    return false;
  }
  // robot_start_pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->robot_start_pose), &(output->robot_start_pose)))
  {
    return false;
  }
  // robot_target_pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->robot_target_pose), &(output->robot_target_pose)))
  {
    return false;
  }
  // num_people
  output->num_people = input->num_people;
  // num_groups
  output->num_groups = input->num_groups;
  // environment
  if (!rosidl_runtime_c__String__copy(
      &(input->environment), &(output->environment)))
  {
    return false;
  }
  return true;
}

simulation_msgs__msg__SceneInfo *
simulation_msgs__msg__SceneInfo__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  simulation_msgs__msg__SceneInfo * msg = (simulation_msgs__msg__SceneInfo *)allocator.allocate(sizeof(simulation_msgs__msg__SceneInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(simulation_msgs__msg__SceneInfo));
  bool success = simulation_msgs__msg__SceneInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
simulation_msgs__msg__SceneInfo__destroy(simulation_msgs__msg__SceneInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    simulation_msgs__msg__SceneInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
simulation_msgs__msg__SceneInfo__Sequence__init(simulation_msgs__msg__SceneInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  simulation_msgs__msg__SceneInfo * data = NULL;

  if (size) {
    data = (simulation_msgs__msg__SceneInfo *)allocator.zero_allocate(size, sizeof(simulation_msgs__msg__SceneInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = simulation_msgs__msg__SceneInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        simulation_msgs__msg__SceneInfo__fini(&data[i - 1]);
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
simulation_msgs__msg__SceneInfo__Sequence__fini(simulation_msgs__msg__SceneInfo__Sequence * array)
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
      simulation_msgs__msg__SceneInfo__fini(&array->data[i]);
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

simulation_msgs__msg__SceneInfo__Sequence *
simulation_msgs__msg__SceneInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  simulation_msgs__msg__SceneInfo__Sequence * array = (simulation_msgs__msg__SceneInfo__Sequence *)allocator.allocate(sizeof(simulation_msgs__msg__SceneInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = simulation_msgs__msg__SceneInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
simulation_msgs__msg__SceneInfo__Sequence__destroy(simulation_msgs__msg__SceneInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    simulation_msgs__msg__SceneInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
simulation_msgs__msg__SceneInfo__Sequence__are_equal(const simulation_msgs__msg__SceneInfo__Sequence * lhs, const simulation_msgs__msg__SceneInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!simulation_msgs__msg__SceneInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
simulation_msgs__msg__SceneInfo__Sequence__copy(
  const simulation_msgs__msg__SceneInfo__Sequence * input,
  simulation_msgs__msg__SceneInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(simulation_msgs__msg__SceneInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    simulation_msgs__msg__SceneInfo * data =
      (simulation_msgs__msg__SceneInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!simulation_msgs__msg__SceneInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          simulation_msgs__msg__SceneInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!simulation_msgs__msg__SceneInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
