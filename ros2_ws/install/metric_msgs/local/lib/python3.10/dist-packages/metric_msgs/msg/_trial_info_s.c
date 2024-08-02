// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from metric_msgs:msg/TrialInfo.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "metric_msgs/msg/detail/trial_info__struct.h"
#include "metric_msgs/msg/detail/trial_info__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "geometry_msgs/msg/detail/pose__functions.h"
// end nested array functions include
ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool builtin_interfaces__msg__time__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * builtin_interfaces__msg__time__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__pose__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__pose__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__pose__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__pose__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__pose__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__pose__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool builtin_interfaces__msg__time__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * builtin_interfaces__msg__time__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool metric_msgs__msg__trial_info__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[38];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("metric_msgs.msg._trial_info.TrialInfo", full_classname_dest, 37) == 0);
  }
  metric_msgs__msg__TrialInfo * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // trial_start
    PyObject * field = PyObject_GetAttrString(_pymsg, "trial_start");
    if (!field) {
      return false;
    }
    if (!builtin_interfaces__msg__time__convert_from_py(field, &ros_message->trial_start)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // timeout_time
    PyObject * field = PyObject_GetAttrString(_pymsg, "timeout_time");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->timeout_time = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // trial_name
    PyObject * field = PyObject_GetAttrString(_pymsg, "trial_name");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->trial_name, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // trial_number
    PyObject * field = PyObject_GetAttrString(_pymsg, "trial_number");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->trial_number = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // num_actors
    PyObject * field = PyObject_GetAttrString(_pymsg, "num_actors");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->num_actors = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // robot_start
    PyObject * field = PyObject_GetAttrString(_pymsg, "robot_start");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__pose__convert_from_py(field, &ros_message->robot_start)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // robot_goal
    PyObject * field = PyObject_GetAttrString(_pymsg, "robot_goal");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__pose__convert_from_py(field, &ros_message->robot_goal)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // dist_to_target
    PyObject * field = PyObject_GetAttrString(_pymsg, "dist_to_target");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->dist_to_target = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // min_dist_to_target
    PyObject * field = PyObject_GetAttrString(_pymsg, "min_dist_to_target");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->min_dist_to_target = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // robot_poses
    PyObject * field = PyObject_GetAttrString(_pymsg, "robot_poses");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'robot_poses'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!geometry_msgs__msg__Pose__Sequence__init(&(ros_message->robot_poses), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create geometry_msgs__msg__Pose__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    geometry_msgs__msg__Pose * dest = ros_message->robot_poses.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!geometry_msgs__msg__pose__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // robot_poses_ts
    PyObject * field = PyObject_GetAttrString(_pymsg, "robot_poses_ts");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'robot_poses_ts'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!builtin_interfaces__msg__Time__Sequence__init(&(ros_message->robot_poses_ts), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create builtin_interfaces__msg__Time__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    builtin_interfaces__msg__Time * dest = ros_message->robot_poses_ts.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!builtin_interfaces__msg__time__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // min_dist_to_ped
    PyObject * field = PyObject_GetAttrString(_pymsg, "min_dist_to_ped");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->min_dist_to_ped = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // robot_on_person_intimate_dist_violations
    PyObject * field = PyObject_GetAttrString(_pymsg, "robot_on_person_intimate_dist_violations");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->robot_on_person_intimate_dist_violations = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // person_on_robot_intimate_dist_violations
    PyObject * field = PyObject_GetAttrString(_pymsg, "person_on_robot_intimate_dist_violations");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->person_on_robot_intimate_dist_violations = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // robot_on_person_personal_dist_violations
    PyObject * field = PyObject_GetAttrString(_pymsg, "robot_on_person_personal_dist_violations");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->robot_on_person_personal_dist_violations = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // person_on_robot_personal_dist_violations
    PyObject * field = PyObject_GetAttrString(_pymsg, "person_on_robot_personal_dist_violations");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->person_on_robot_personal_dist_violations = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // robot_on_person_collisions
    PyObject * field = PyObject_GetAttrString(_pymsg, "robot_on_person_collisions");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->robot_on_person_collisions = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // person_on_robot_collisions
    PyObject * field = PyObject_GetAttrString(_pymsg, "person_on_robot_collisions");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->person_on_robot_collisions = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // obj_collisions
    PyObject * field = PyObject_GetAttrString(_pymsg, "obj_collisions");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->obj_collisions = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // path_length
    PyObject * field = PyObject_GetAttrString(_pymsg, "path_length");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->path_length = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // path_irregularity
    PyObject * field = PyObject_GetAttrString(_pymsg, "path_irregularity");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->path_irregularity = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // time_not_moving
    PyObject * field = PyObject_GetAttrString(_pymsg, "time_not_moving");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->time_not_moving = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // time_in_personal_space
    PyObject * field = PyObject_GetAttrString(_pymsg, "time_in_personal_space");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->time_in_personal_space = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // minimum_time_to_collision
    PyObject * field = PyObject_GetAttrString(_pymsg, "minimum_time_to_collision");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->minimum_time_to_collision = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // movement_jerk
    PyObject * field = PyObject_GetAttrString(_pymsg, "movement_jerk");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->movement_jerk = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * metric_msgs__msg__trial_info__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of TrialInfo */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("metric_msgs.msg._trial_info");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "TrialInfo");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  metric_msgs__msg__TrialInfo * ros_message = (metric_msgs__msg__TrialInfo *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // trial_start
    PyObject * field = NULL;
    field = builtin_interfaces__msg__time__convert_to_py(&ros_message->trial_start);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "trial_start", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // timeout_time
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->timeout_time);
    {
      int rc = PyObject_SetAttrString(_pymessage, "timeout_time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // trial_name
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->trial_name.data,
      strlen(ros_message->trial_name.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "trial_name", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // trial_number
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->trial_number);
    {
      int rc = PyObject_SetAttrString(_pymessage, "trial_number", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // num_actors
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->num_actors);
    {
      int rc = PyObject_SetAttrString(_pymessage, "num_actors", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // robot_start
    PyObject * field = NULL;
    field = geometry_msgs__msg__pose__convert_to_py(&ros_message->robot_start);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "robot_start", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // robot_goal
    PyObject * field = NULL;
    field = geometry_msgs__msg__pose__convert_to_py(&ros_message->robot_goal);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "robot_goal", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // dist_to_target
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->dist_to_target);
    {
      int rc = PyObject_SetAttrString(_pymessage, "dist_to_target", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // min_dist_to_target
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->min_dist_to_target);
    {
      int rc = PyObject_SetAttrString(_pymessage, "min_dist_to_target", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // robot_poses
    PyObject * field = NULL;
    size_t size = ros_message->robot_poses.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    geometry_msgs__msg__Pose * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->robot_poses.data[i]);
      PyObject * pyitem = geometry_msgs__msg__pose__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "robot_poses", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // robot_poses_ts
    PyObject * field = NULL;
    size_t size = ros_message->robot_poses_ts.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    builtin_interfaces__msg__Time * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->robot_poses_ts.data[i]);
      PyObject * pyitem = builtin_interfaces__msg__time__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "robot_poses_ts", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // min_dist_to_ped
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->min_dist_to_ped);
    {
      int rc = PyObject_SetAttrString(_pymessage, "min_dist_to_ped", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // robot_on_person_intimate_dist_violations
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->robot_on_person_intimate_dist_violations);
    {
      int rc = PyObject_SetAttrString(_pymessage, "robot_on_person_intimate_dist_violations", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // person_on_robot_intimate_dist_violations
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->person_on_robot_intimate_dist_violations);
    {
      int rc = PyObject_SetAttrString(_pymessage, "person_on_robot_intimate_dist_violations", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // robot_on_person_personal_dist_violations
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->robot_on_person_personal_dist_violations);
    {
      int rc = PyObject_SetAttrString(_pymessage, "robot_on_person_personal_dist_violations", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // person_on_robot_personal_dist_violations
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->person_on_robot_personal_dist_violations);
    {
      int rc = PyObject_SetAttrString(_pymessage, "person_on_robot_personal_dist_violations", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // robot_on_person_collisions
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->robot_on_person_collisions);
    {
      int rc = PyObject_SetAttrString(_pymessage, "robot_on_person_collisions", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // person_on_robot_collisions
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->person_on_robot_collisions);
    {
      int rc = PyObject_SetAttrString(_pymessage, "person_on_robot_collisions", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // obj_collisions
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->obj_collisions);
    {
      int rc = PyObject_SetAttrString(_pymessage, "obj_collisions", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // path_length
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->path_length);
    {
      int rc = PyObject_SetAttrString(_pymessage, "path_length", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // path_irregularity
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->path_irregularity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "path_irregularity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // time_not_moving
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->time_not_moving);
    {
      int rc = PyObject_SetAttrString(_pymessage, "time_not_moving", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // time_in_personal_space
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->time_in_personal_space);
    {
      int rc = PyObject_SetAttrString(_pymessage, "time_in_personal_space", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // minimum_time_to_collision
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->minimum_time_to_collision);
    {
      int rc = PyObject_SetAttrString(_pymessage, "minimum_time_to_collision", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // movement_jerk
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->movement_jerk);
    {
      int rc = PyObject_SetAttrString(_pymessage, "movement_jerk", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
