# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_assets_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED assets_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(assets_FOUND FALSE)
  elseif(NOT assets_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(assets_FOUND FALSE)
  endif()
  return()
endif()
set(_assets_CONFIG_INCLUDED TRUE)

# output package information
if(NOT assets_FIND_QUIETLY)
  message(STATUS "Found assets: 0.0.0 (${assets_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'assets' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${assets_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(assets_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${assets_DIR}/${_extra}")
endforeach()
