#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "graph_msgs::graph_msgs__rosidl_generator_py" for configuration "Release"
set_property(TARGET graph_msgs::graph_msgs__rosidl_generator_py APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(graph_msgs::graph_msgs__rosidl_generator_py PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libgraph_msgs__rosidl_generator_py.so"
  IMPORTED_SONAME_RELEASE "libgraph_msgs__rosidl_generator_py.so"
  )

list(APPEND _cmake_import_check_targets graph_msgs::graph_msgs__rosidl_generator_py )
list(APPEND _cmake_import_check_files_for_graph_msgs::graph_msgs__rosidl_generator_py "${_IMPORT_PREFIX}/lib/libgraph_msgs__rosidl_generator_py.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
