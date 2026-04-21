# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ascamera_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ascamera_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ascamera_FOUND FALSE)
  elseif(NOT ascamera_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ascamera_FOUND FALSE)
  endif()
  return()
endif()
set(_ascamera_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ascamera_FIND_QUIETLY)
  message(STATUS "Found ascamera: 1.0.0 (${ascamera_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ascamera' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ascamera_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ascamera_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${ascamera_DIR}/${_extra}")
endforeach()
