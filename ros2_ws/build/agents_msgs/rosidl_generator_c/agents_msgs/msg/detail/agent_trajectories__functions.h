// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from agents_msgs:msg/AgentTrajectories.idl
// generated code does not contain a copyright notice

#ifndef AGENTS_MSGS__MSG__DETAIL__AGENT_TRAJECTORIES__FUNCTIONS_H_
#define AGENTS_MSGS__MSG__DETAIL__AGENT_TRAJECTORIES__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "agents_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "agents_msgs/msg/detail/agent_trajectories__struct.h"

/// Initialize msg/AgentTrajectories message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * agents_msgs__msg__AgentTrajectories
 * )) before or use
 * agents_msgs__msg__AgentTrajectories__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_agents_msgs
bool
agents_msgs__msg__AgentTrajectories__init(agents_msgs__msg__AgentTrajectories * msg);

/// Finalize msg/AgentTrajectories message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_agents_msgs
void
agents_msgs__msg__AgentTrajectories__fini(agents_msgs__msg__AgentTrajectories * msg);

/// Create msg/AgentTrajectories message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * agents_msgs__msg__AgentTrajectories__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_agents_msgs
agents_msgs__msg__AgentTrajectories *
agents_msgs__msg__AgentTrajectories__create();

/// Destroy msg/AgentTrajectories message.
/**
 * It calls
 * agents_msgs__msg__AgentTrajectories__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_agents_msgs
void
agents_msgs__msg__AgentTrajectories__destroy(agents_msgs__msg__AgentTrajectories * msg);

/// Check for msg/AgentTrajectories message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_agents_msgs
bool
agents_msgs__msg__AgentTrajectories__are_equal(const agents_msgs__msg__AgentTrajectories * lhs, const agents_msgs__msg__AgentTrajectories * rhs);

/// Copy a msg/AgentTrajectories message.
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
ROSIDL_GENERATOR_C_PUBLIC_agents_msgs
bool
agents_msgs__msg__AgentTrajectories__copy(
  const agents_msgs__msg__AgentTrajectories * input,
  agents_msgs__msg__AgentTrajectories * output);

/// Initialize array of msg/AgentTrajectories messages.
/**
 * It allocates the memory for the number of elements and calls
 * agents_msgs__msg__AgentTrajectories__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_agents_msgs
bool
agents_msgs__msg__AgentTrajectories__Sequence__init(agents_msgs__msg__AgentTrajectories__Sequence * array, size_t size);

/// Finalize array of msg/AgentTrajectories messages.
/**
 * It calls
 * agents_msgs__msg__AgentTrajectories__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_agents_msgs
void
agents_msgs__msg__AgentTrajectories__Sequence__fini(agents_msgs__msg__AgentTrajectories__Sequence * array);

/// Create array of msg/AgentTrajectories messages.
/**
 * It allocates the memory for the array and calls
 * agents_msgs__msg__AgentTrajectories__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_agents_msgs
agents_msgs__msg__AgentTrajectories__Sequence *
agents_msgs__msg__AgentTrajectories__Sequence__create(size_t size);

/// Destroy array of msg/AgentTrajectories messages.
/**
 * It calls
 * agents_msgs__msg__AgentTrajectories__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_agents_msgs
void
agents_msgs__msg__AgentTrajectories__Sequence__destroy(agents_msgs__msg__AgentTrajectories__Sequence * array);

/// Check for msg/AgentTrajectories message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_agents_msgs
bool
agents_msgs__msg__AgentTrajectories__Sequence__are_equal(const agents_msgs__msg__AgentTrajectories__Sequence * lhs, const agents_msgs__msg__AgentTrajectories__Sequence * rhs);

/// Copy an array of msg/AgentTrajectories messages.
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
ROSIDL_GENERATOR_C_PUBLIC_agents_msgs
bool
agents_msgs__msg__AgentTrajectories__Sequence__copy(
  const agents_msgs__msg__AgentTrajectories__Sequence * input,
  agents_msgs__msg__AgentTrajectories__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // AGENTS_MSGS__MSG__DETAIL__AGENT_TRAJECTORIES__FUNCTIONS_H_
