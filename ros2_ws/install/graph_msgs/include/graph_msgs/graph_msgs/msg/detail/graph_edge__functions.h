// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from graph_msgs:msg/GraphEdge.idl
// generated code does not contain a copyright notice

#ifndef GRAPH_MSGS__MSG__DETAIL__GRAPH_EDGE__FUNCTIONS_H_
#define GRAPH_MSGS__MSG__DETAIL__GRAPH_EDGE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "graph_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "graph_msgs/msg/detail/graph_edge__struct.h"

/// Initialize msg/GraphEdge message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * graph_msgs__msg__GraphEdge
 * )) before or use
 * graph_msgs__msg__GraphEdge__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_graph_msgs
bool
graph_msgs__msg__GraphEdge__init(graph_msgs__msg__GraphEdge * msg);

/// Finalize msg/GraphEdge message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_graph_msgs
void
graph_msgs__msg__GraphEdge__fini(graph_msgs__msg__GraphEdge * msg);

/// Create msg/GraphEdge message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * graph_msgs__msg__GraphEdge__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_graph_msgs
graph_msgs__msg__GraphEdge *
graph_msgs__msg__GraphEdge__create();

/// Destroy msg/GraphEdge message.
/**
 * It calls
 * graph_msgs__msg__GraphEdge__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_graph_msgs
void
graph_msgs__msg__GraphEdge__destroy(graph_msgs__msg__GraphEdge * msg);

/// Check for msg/GraphEdge message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_graph_msgs
bool
graph_msgs__msg__GraphEdge__are_equal(const graph_msgs__msg__GraphEdge * lhs, const graph_msgs__msg__GraphEdge * rhs);

/// Copy a msg/GraphEdge message.
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
ROSIDL_GENERATOR_C_PUBLIC_graph_msgs
bool
graph_msgs__msg__GraphEdge__copy(
  const graph_msgs__msg__GraphEdge * input,
  graph_msgs__msg__GraphEdge * output);

/// Initialize array of msg/GraphEdge messages.
/**
 * It allocates the memory for the number of elements and calls
 * graph_msgs__msg__GraphEdge__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_graph_msgs
bool
graph_msgs__msg__GraphEdge__Sequence__init(graph_msgs__msg__GraphEdge__Sequence * array, size_t size);

/// Finalize array of msg/GraphEdge messages.
/**
 * It calls
 * graph_msgs__msg__GraphEdge__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_graph_msgs
void
graph_msgs__msg__GraphEdge__Sequence__fini(graph_msgs__msg__GraphEdge__Sequence * array);

/// Create array of msg/GraphEdge messages.
/**
 * It allocates the memory for the array and calls
 * graph_msgs__msg__GraphEdge__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_graph_msgs
graph_msgs__msg__GraphEdge__Sequence *
graph_msgs__msg__GraphEdge__Sequence__create(size_t size);

/// Destroy array of msg/GraphEdge messages.
/**
 * It calls
 * graph_msgs__msg__GraphEdge__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_graph_msgs
void
graph_msgs__msg__GraphEdge__Sequence__destroy(graph_msgs__msg__GraphEdge__Sequence * array);

/// Check for msg/GraphEdge message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_graph_msgs
bool
graph_msgs__msg__GraphEdge__Sequence__are_equal(const graph_msgs__msg__GraphEdge__Sequence * lhs, const graph_msgs__msg__GraphEdge__Sequence * rhs);

/// Copy an array of msg/GraphEdge messages.
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
ROSIDL_GENERATOR_C_PUBLIC_graph_msgs
bool
graph_msgs__msg__GraphEdge__Sequence__copy(
  const graph_msgs__msg__GraphEdge__Sequence * input,
  graph_msgs__msg__GraphEdge__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // GRAPH_MSGS__MSG__DETAIL__GRAPH_EDGE__FUNCTIONS_H_
