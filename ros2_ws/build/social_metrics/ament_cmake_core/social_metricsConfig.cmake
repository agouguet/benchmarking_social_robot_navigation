# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_social_metrics_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED social_metrics_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(social_metrics_FOUND FALSE)
  elseif(NOT social_metrics_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(social_metrics_FOUND FALSE)
  endif()
  return()
endif()
set(_social_metrics_CONFIG_INCLUDED TRUE)

# output package information
if(NOT social_metrics_FIND_QUIETLY)
  message(STATUS "Found social_metrics: 0.0.0 (${social_metrics_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'social_metrics' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${social_metrics_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(social_metrics_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${social_metrics_DIR}/${_extra}")
endforeach()
