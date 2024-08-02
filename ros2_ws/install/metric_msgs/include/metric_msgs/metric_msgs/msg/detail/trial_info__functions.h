// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from metric_msgs:msg/TrialInfo.idl
// generated code does not contain a copyright notice

#ifndef METRIC_MSGS__MSG__DETAIL__TRIAL_INFO__FUNCTIONS_H_
#define METRIC_MSGS__MSG__DETAIL__TRIAL_INFO__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "metric_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "metric_msgs/msg/detail/trial_info__struct.h"

/// Initialize msg/TrialInfo message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * metric_msgs__msg__TrialInfo
 * )) before or use
 * metric_msgs__msg__TrialInfo__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_metric_msgs
bool
metric_msgs__msg__TrialInfo__init(metric_msgs__msg__TrialInfo * msg);

/// Finalize msg/TrialInfo message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_metric_msgs
void
metric_msgs__msg__TrialInfo__fini(metric_msgs__msg__TrialInfo * msg);

/// Create msg/TrialInfo message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * metric_msgs__msg__TrialInfo__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_metric_msgs
metric_msgs__msg__TrialInfo *
metric_msgs__msg__TrialInfo__create();

/// Destroy msg/TrialInfo message.
/**
 * It calls
 * metric_msgs__msg__TrialInfo__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_metric_msgs
void
metric_msgs__msg__TrialInfo__destroy(metric_msgs__msg__TrialInfo * msg);

/// Check for msg/TrialInfo message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_metric_msgs
bool
metric_msgs__msg__TrialInfo__are_equal(const metric_msgs__msg__TrialInfo * lhs, const metric_msgs__msg__TrialInfo * rhs);

/// Copy a msg/TrialInfo message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_metric_msgs
bool
metric_msgs__msg__TrialInfo__copy(
  const metric_msgs__msg__TrialInfo * input,
  metric_msgs__msg__TrialInfo * output);

/// Initialize array of msg/TrialInfo messages.
/**
 * It allocates the memory for the number of elements and calls
 * metric_msgs__msg__TrialInfo__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_metric_msgs
bool
metric_msgs__msg__TrialInfo__Sequence__init(metric_msgs__msg__TrialInfo__Sequence * array, size_t size);

/// Finalize array of msg/TrialInfo messages.
/**
 * It calls
 * metric_msgs__msg__TrialInfo__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_metric_msgs
void
metric_msgs__msg__TrialInfo__Sequence__fini(metric_msgs__msg__TrialInfo__Sequence * array);

/// Create array of msg/TrialInfo messages.
/**
 * It allocates the memory for the array and calls
 * metric_msgs__msg__TrialInfo__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_metric_msgs
metric_msgs__msg__TrialInfo__Sequence *
metric_msgs__msg__TrialInfo__Sequence__create(size_t size);

/// Destroy array of msg/TrialInfo messages.
/**
 * It calls
 * metric_msgs__msg__TrialInfo__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_metric_msgs
void
metric_msgs__msg__TrialInfo__Sequence__destroy(metric_msgs__msg__TrialInfo__Sequence * array);

/// Check for msg/TrialInfo message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_metric_msgs
bool
metric_msgs__msg__TrialInfo__Sequence__are_equal(const metric_msgs__msg__TrialInfo__Sequence * lhs, const metric_msgs__msg__TrialInfo__Sequence * rhs);

/// Copy an array of msg/TrialInfo messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_metric_msgs
bool
metric_msgs__msg__TrialInfo__Sequence__copy(
  const metric_msgs__msg__TrialInfo__Sequence * input,
  metric_msgs__msg__TrialInfo__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // METRIC_MSGS__MSG__DETAIL__TRIAL_INFO__FUNCTIONS_H_
