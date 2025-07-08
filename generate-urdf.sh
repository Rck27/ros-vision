#!/bin/bash
modelpath='src/sjtu_drone/sjtu_drone_description';
ros2 run xacro xacro -o $modelpath/urdf/sjtu_drone.urdf ${modelpath}/urdf/sjtu_drone.urdf.xacro params_path:="$(ros2 pkg prefix sjtu_drone_bringup)/share/sjtu_drone_bringup/config/drone.yaml"
gz sdf -p ${modelpath}/urdf/sjtu_drone.urdf > ${modelpath}/models/sjtu_drone/model.sdf 
colcon build