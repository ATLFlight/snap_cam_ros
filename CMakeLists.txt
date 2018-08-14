# /****************************************************************************
#  *   Copyright (c) 2017 Michael Shomin. All rights reserved.
#  *
#  * Redistribution and use in source and binary forms, with or without
#  * modification, are permitted provided that the following conditions
#  * are met:
#  *
#  * 1. Redistributions of source code must retain the above copyright
#  *    notice, this list of conditions and the following disclaimer.
#  * 2. Redistributions in binary form must reproduce the above copyright
#  *    notice, this list of conditions and the following disclaimer in
#  *    the documentation and/or other materials provided with the
#  *    distribution.
#  * 3. Neither the name ATLFlight nor the names of its contributors may be
#  *    used to endorse or promote products derived from this software
#  *    without specific prior written permission.
#  *
#  * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
#  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
#  * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
#  * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  * POSSIBILITY OF SUCH DAMAGE.
#  *
#  * In addition Supplemental Terms apply.  See the SUPPLEMENTAL file. 
#  ****************************************************************************/


cmake_minimum_required(VERSION 2.8.3)
project(snap_cam_ros)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  camera_info_manager
  dynamic_reconfigure
  image_transport
  nodelet
  sensor_msgs
  snap_msgs
  )

generate_dynamic_reconfigure_options(cfg/SnapCam.cfg)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    camera_info_manager
    dynamic_reconfigure
    image_transport
    nodelet
    sensor_msgs
    snap_msgs
  LIBRARIES snap_cam_nodelet
  )

add_definitions(-std=c++11) 

# Use the APQ8096 flag for Snapdragon 8x96 processors
if ("${QC_SOC_TARGET}" STREQUAL "APQ8096")
  add_definitions( -DQC_SOC_TARGET_APQ8096 )
  message("Building for APQ8096")
  # Use the TOF flag for TOF Support on Snapdragon 8x96 processors
  if ("${QC_TOF_BUILD}" STREQUAL "TOF")
    add_definitions( -DQC_TOF_BUILD )
    message("Building for APQ8096 TOF")
  endif()

# Use the APQ8074 flag for Snapdragon 8x74 processors
# Like the Snapdragon Flight board
else()
  add_definitions( -DQC_SOC_TARGET_APQ8074 )
  message("Building for APQ8074")
endif()

include_directories(include)
include_directories(snap_cam_manager/src)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_library(SnapdragonCameraManager
    snap_cam_manager/src/SnapdragonCameraManager.cpp
    snap_cam_manager/src/SnapdragonCameraUtil.cpp
    )
target_link_libraries(SnapdragonCameraManager
  camera
)

add_library(snap_cam src/snap_cam.cpp)
target_link_libraries(snap_cam
  ${catkin_LIBRARIES}
  SnapdragonCameraManager
)
add_dependencies(snap_cam ${PROJECT_NAME}_gencfg)
add_dependencies(snap_cam ${catkin_EXPORTED_TARGETS})



add_executable(snap_cam_node src/snap_cam_node.cpp)
target_link_libraries(snap_cam_node
  ${catkin_LIBRARIES}
  SnapdragonCameraManager
  snap_cam
)
add_dependencies(snap_cam_node ${PROJECT_NAME}_gencfg)
add_dependencies(snap_cam_node ${catkin_EXPORTED_TARGETS})

add_library(snap_cam_nodelet src/snap_cam_nodelet.cpp)
add_dependencies(snap_cam_nodelet ${snap_cam_ros_EXPORTED_TARGETS})
add_dependencies(snap_cam_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(snap_cam_nodelet
  ${Boost_LIBRARIES} 
  ${catkin_LIBRARIES}
  SnapdragonCameraManager
  snap_cam
  )
add_dependencies(snap_cam_nodelet ${PROJECT_NAME}_gencfg)


add_library(stereo_nodelet src/stereo_nodelet.cpp)
add_dependencies(stereo_nodelet ${snap_cam_ros_EXPORTED_TARGETS})
add_dependencies(stereo_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(stereo_nodelet
  ${Boost_LIBRARIES} 
  ${catkin_LIBRARIES}
  SnapdragonCameraManager
  snap_cam
  )
add_dependencies(stereo_nodelet ${PROJECT_NAME}_gencfg)

add_custom_command(
TARGET snap_cam_node
COMMAND echo "Setting the UID bit for the node to run with root privileges"
COMMAND sudo chown root ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/snap_cam_node
COMMAND sudo chmod +s ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/snap_cam_node
)

install(TARGETS snap_cam snap_cam_node snap_cam_nodelet stereo_nodelet SnapdragonCameraManager
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(FILES snap_cam_nodelet.xml stereo_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
