# Copyright (C) 2006-2022 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

cmake_minimum_required(VERSION 3.12)

project(ergocub-software
        VERSION 0.5.0)

# Give error if add_dependencies is called on a non-existing target
if(POLICY CMP0046)
  cmake_policy(SET CMP0046 NEW)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(AddUninstallTarget)

option(BUILD_TESTING "Run tests for the generated models" OFF)
option(COMPILE_ergoCubEmotions "Compile the module controlling the ergoCub emotions" OFF)

set(BUILD_PREFIX "ergoCub")
add_subdirectory(urdf)

# Install empty ergoCub file in <prefix>/share/ament_index/resource_index/packages
# so that ergoCub package is found by ROS2
# See https://github.com/robotology/icub-models/issues/177
# and https://github.com/icub-tech-iit/ergocub-software/issues/59
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/ergoCub_empty_file "")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/ergoCub_empty_file
        DESTINATION share/ament_index/resource_index/packages
        RENAME ergoCub)

# TODO to be defined
if (BUILD_TESTING)
  include( CTest )
  enable_testing()
  add_subdirectory(tests)
endif()

if(COMPILE_ergoCubEmotions)
  add_subdirectory(src)
endif()


