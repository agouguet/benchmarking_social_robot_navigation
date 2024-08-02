// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from metric_msgs:msg/TrialInfo.idl
// generated code does not contain a copyright notice
#include "metric_msgs/msg/detail/trial_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `trial_start`
// Member `robot_poses_ts`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `trial_name`
#include "rosidl_runtime_c/string_functions.h"
// Member `robot_start`
// Member `robot_goal`
// Member `robot_poses`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
metric_msgs__msg__TrialInfo__init(metric_msgs__msg__TrialInfo * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    metric_msgs__msg__TrialInfo__fini(msg);
    return false;
  }
  // trial_start
  if (!builtin_interfaces__msg__Time__init(&msg->trial_start)) {
    metric_msgs__msg__TrialInfo__fini(msg);
    return false;
  }
  // timeout_time
  // trial_name
  if (!rosidl_runtime_c__String__init(&msg->trial_name)) {
    metric_msgs__msg__TrialInfo__fini(msg);
    return false;
  }
  // trial_number
  // num_actors
  // robot_start
  if (!geometry_msgs__msg__Pose__init(&msg->robot_start)) {
    metric_msgs__msg__TrialInfo__fini(msg);
    return false;
  }
  // robot_goal
  if (!geometry_msgs__msg__Pose__init(&msg->robot_goal)) {
    metric_msgs__msg__TrialInfo__fini(msg);
    return false;
  }
  // dist_to_target
  // min_dist_to_target
  // robot_poses
  if (!geometry_msgs__msg__Pose__Sequence__init(&msg->robot_poses, 0)) {
    metric_msgs__msg__TrialInfo__fini(msg);
    return false;
  }
  // robot_poses_ts
  if (!builtin_interfaces__msg__Time__Sequence__init(&msg->robot_poses_ts, 0)) {
    metric_msgs__msg__TrialInfo__fini(msg);
    return false;
  }
  // min_dist_to_ped
  // robot_on_person_intimate_dist_violations
  // person_on_robot_intimate_dist_violations
  // robot_on_person_personal_dist_violations
  // person_on_robot_personal_dist_violations
  // robot_on_person_collisions
  // person_on_robot_collisions
  // obj_collisions
  // path_length
  // path_irregularity
  // time_not_moving
  // time_in_personal_space
  // minimum_time_to_collision
  // movement_jerk
  return true;
}

void
metric_msgs__msg__TrialInfo__fini(metric_msgs__msg__TrialInfo * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // trial_start
  builtin_interfaces__msg__Time__fini(&msg->trial_start);
  // timeout_time
  // trial_name
  rosidl_runtime_c__String__fini(&msg->trial_name);
  // trial_number
  // num_actors
  // robot_start
  geometry_msgs__msg__Pose__fini(&msg->robot_start);
  // robot_goal
  geometry_msgs__msg__Pose__fini(&msg->robot_goal);
  // dist_to_target
  // min_dist_to_target
  // robot_poses
  geometry_msgs__msg__Pose__Sequence__fini(&msg->robot_poses);
  // robot_poses_ts
  builtin_interfaces__msg__Time__Sequence__fini(&msg->robot_poses_ts);
  // min_dist_to_ped
  // robot_on_person_intimate_dist_violations
  // person_on_robot_intimate_dist_violations
  // robot_on_person_personal_dist_violations
  // person_on_robot_personal_dist_violations
  // robot_on_person_collisions
  // person_on_robot_collisions
  // obj_collisions
  // path_length
  // path_irregularity
  // time_not_moving
  // time_in_personal_space
  // minimum_time_to_collision
  // movement_jerk
}

bool
metric_msgs__msg__TrialInfo__are_equal(const metric_msgs__msg__TrialInfo * lhs, const metric_msgs__msg__TrialInfo * rhs)
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
  // trial_start
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->trial_start), &(rhs->trial_start)))
  {
    return false;
  }
  // timeout_time
  if (lhs->timeout_time != rhs->timeout_time) {
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
  // num_actors
  if (lhs->num_actors != rhs->num_actors) {
    return false;
  }
  // robot_start
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->robot_start), &(rhs->robot_start)))
  {
    return false;
  }
  // robot_goal
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->robot_goal), &(rhs->robot_goal)))
  {
    return false;
  }
  // dist_to_target
  if (lhs->dist_to_target != rhs->dist_to_target) {
    return false;
  }
  // min_dist_to_target
  if (lhs->min_dist_to_target != rhs->min_dist_to_target) {
    return false;
  }
  // robot_poses
  if (!geometry_msgs__msg__Pose__Sequence__are_equal(
      &(lhs->robot_poses), &(rhs->robot_poses)))
  {
    return false;
  }
  // robot_poses_ts
  if (!builtin_interfaces__msg__Time__Sequence__are_equal(
      &(lhs->robot_poses_ts), &(rhs->robot_poses_ts)))
  {
    return false;
  }
  // min_dist_to_ped
  if (lhs->min_dist_to_ped != rhs->min_dist_to_ped) {
    return false;
  }
  // robot_on_person_intimate_dist_violations
  if (lhs->robot_on_person_intimate_dist_violations != rhs->robot_on_person_intimate_dist_violations) {
    return false;
  }
  // person_on_robot_intimate_dist_violations
  if (lhs->person_on_robot_intimate_dist_violations != rhs->person_on_robot_intimate_dist_violations) {
    return false;
  }
  // robot_on_person_personal_dist_violations
  if (lhs->robot_on_person_personal_dist_violations != rhs->robot_on_person_personal_dist_violations) {
    return false;
  }
  // person_on_robot_personal_dist_violations
  if (lhs->person_on_robot_personal_dist_violations != rhs->person_on_robot_personal_dist_violations) {
    return false;
  }
  // robot_on_person_collisions
  if (lhs->robot_on_person_collisions != rhs->robot_on_person_collisions) {
    return false;
  }
  // person_on_robot_collisions
  if (lhs->person_on_robot_collisions != rhs->person_on_robot_collisions) {
    return false;
  }
  // obj_collisions
  if (lhs->obj_collisions != rhs->obj_collisions) {
    return false;
  }
  // path_length
  if (lhs->path_length != rhs->path_length) {
    return false;
  }
  // path_irregularity
  if (lhs->path_irregularity != rhs->path_irregularity) {
    return false;
  }
  // time_not_moving
  if (lhs->time_not_moving != rhs->time_not_moving) {
    return false;
  }
  // time_in_personal_space
  if (lhs->time_in_personal_space != rhs->time_in_personal_space) {
    return false;
  }
  // minimum_time_to_collision
  if (lhs->minimum_time_to_collision != rhs->minimum_time_to_collision) {
    return false;
  }
  // movement_jerk
  if (lhs->movement_jerk != rhs->movement_jerk) {
    return false;
  }
  return true;
}

bool
metric_msgs__msg__TrialInfo__copy(
  const metric_msgs__msg__TrialInfo * input,
  metric_msgs__msg__TrialInfo * output)
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
  // trial_start
  if (!builtin_interfaces__msg__Time__copy(
      &(input->trial_start), &(output->trial_start)))
  {
    return false;
  }
  // timeout_time
  output->timeout_time = input->timeout_time;
  // trial_name
  if (!rosidl_runtime_c__String__copy(
      &(input->trial_name), &(output->trial_name)))
  {
    return false;
  }
  // trial_number
  output->trial_number = input->trial_number;
  // num_actors
  output->num_actors = input->num_actors;
  // robot_start
  if (!geometry_msgs__msg__Pose__copy(
      &(input->robot_start), &(output->robot_start)))
  {
    return false;
  }
  // robot_goal
  if (!geometry_msgs__msg__Pose__copy(
      &(input->robot_goal), &(output->robot_goal)))
  {
    return false;
  }
  // dist_to_target
  output->dist_to_target = input->dist_to_target;
  // min_dist_to_target
  output->min_dist_to_target = input->min_dist_to_target;
  // robot_poses
  if (!geometry_msgs__msg__Pose__Sequence__copy(
      &(input->robot_poses), &(output->robot_poses)))
  {
    return false;
  }
  // robot_poses_ts
  if (!builtin_interfaces__msg__Time__Sequence__copy(
      &(input->robot_poses_ts), &(output->robot_poses_ts)))
  {
    return false;
  }
  // min_dist_to_ped
  output->min_dist_to_ped = input->min_dist_to_ped;
  // robot_on_person_intimate_dist_violations
  output->robot_on_person_intimate_dist_violations = input->robot_on_person_intimate_dist_violations;
  // person_on_robot_intimate_dist_violations
  output->person_on_robot_intimate_dist_violations = input->person_on_robot_intimate_dist_violations;
  // robot_on_person_personal_dist_violations
  output->robot_on_person_personal_dist_violations = input->robot_on_person_personal_dist_violations;
  // person_on_robot_personal_dist_violations
  output->person_on_robot_personal_dist_violations = input->person_on_robot_personal_dist_violations;
  // robot_on_person_collisions
  output->robot_on_person_collisions = input->robot_on_person_collisions;
  // person_on_robot_collisions
  output->person_on_robot_collisions = input->person_on_robot_collisions;
  // obj_collisions
  output->obj_collisions = input->obj_collisions;
  // path_length
  output->path_length = input->path_length;
  // path_irregularity
  output->path_irregularity = input->path_irregularity;
  // time_not_moving
  output->time_not_moving = input->time_not_moving;
  // time_in_personal_space
  output->time_in_personal_space = input->time_in_personal_space;
  // minimum_time_to_collision
  output->minimum_time_to_collision = input->minimum_time_to_collision;
  // movement_jerk
  output->movement_jerk = input->movement_jerk;
  return true;
}

metric_msgs__msg__TrialInfo *
metric_msgs__msg__TrialInfo__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  metric_msgs__msg__TrialInfo * msg = (metric_msgs__msg__TrialInfo *)allocator.allocate(sizeof(metric_msgs__msg__TrialInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(metric_msgs__msg__TrialInfo));
  bool success = metric_msgs__msg__TrialInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
metric_msgs__msg__TrialInfo__destroy(metric_msgs__msg__TrialInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    metric_msgs__msg__TrialInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
metric_msgs__msg__TrialInfo__Sequence__init(metric_msgs__msg__TrialInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  metric_msgs__msg__TrialInfo * data = NULL;

  if (size) {
    data = (metric_msgs__msg__TrialInfo *)allocator.zero_allocate(size, sizeof(metric_msgs__msg__TrialInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = metric_msgs__msg__TrialInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        metric_msgs__msg__TrialInfo__fini(&data[i - 1]);
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
metric_msgs__msg__TrialInfo__Sequence__fini(metric_msgs__msg__TrialInfo__Sequence * array)
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
      metric_msgs__msg__TrialInfo__fini(&array->data[i]);
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

metric_msgs__msg__TrialInfo__Sequence *
metric_msgs__msg__TrialInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  metric_msgs__msg__TrialInfo__Sequence * array = (metric_msgs__msg__TrialInfo__Sequence *)allocator.allocate(sizeof(metric_msgs__msg__TrialInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = metric_msgs__msg__TrialInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
metric_msgs__msg__TrialInfo__Sequence__destroy(metric_msgs__msg__TrialInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    metric_msgs__msg__TrialInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
metric_msgs__msg__TrialInfo__Sequence__are_equal(const metric_msgs__msg__TrialInfo__Sequence * lhs, const metric_msgs__msg__TrialInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!metric_msgs__msg__TrialInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
metric_msgs__msg__TrialInfo__Sequence__copy(
  const metric_msgs__msg__TrialInfo__Sequence * input,
  metric_msgs__msg__TrialInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(metric_msgs__msg__TrialInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    metric_msgs__msg__TrialInfo * data =
      (metric_msgs__msg__TrialInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!metric_msgs__msg__TrialInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          metric_msgs__msg__TrialInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!metric_msgs__msg__TrialInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
