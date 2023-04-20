# Copyright 2022 CNRS
# Author: Florent Lamiraux
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:

# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Install launch files in launch directory
INSTALL_DEMO_LAUNCH_FILES(ur3 pointing
  LAUNCHFILES
  launch/demo.launch
  launch/demo_calib.launch
  launch/estimation.launch
  launch/estimation_realsense.launch
  launch/load_ur.launch
  launch/load_ur3e.launch
  launch/simulation.launch
  launch/ur3_upload.launch
  launch/world_setup.launch
  )

INSTALL(FILES
  supervisor.py
  supervisor_gripper.py
  yaml/demo-kapla.yaml
  yaml/demo-april-tag-plank.yaml
  yaml/demo-plaque-ur3.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ur3/pointing
  )
  
INSTALL(PROGRAMS
    gripper_control.py
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ur3/pointing
    )  
  
INSTALL(FILES
  urdf/kapla.urdf
  urdf/april-tag-plank.urdf
  urdf/plaque-ur3.urdf
  urdf/robot.urdf.xacro
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ur3/pointing/urdf
  )
   
  
INSTALL(FILES
  srdf/kapla.srdf
  srdf/april-tag-plank.srdf
  srdf/plaque-ur3.srdf
  srdf/ur3_robot.srdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ur3/pointing/srdf
  )

INSTALL(FILES
  srdf/ur3_robot.srdf
  srdf/plaque-ur3.srdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srdf
  )  

INSTALL(FILES
  meshes/IMC.stl
  meshes/gripper_2f85_opened.stl
  meshes/robotiq_85_coupler.stl
  meshes/ur3_d435_mount.stl
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ur3/pointing/meshes
  )
  
 
INSTALL(DIRECTORY config DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}/ur3/pointing)
