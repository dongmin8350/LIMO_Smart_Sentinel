#!/bin/bash
cd /home/wego/jetson-inference
exec /home/wego/team_ws/psh030917/nface_ws/install/ros_deep_learning/lib/ros_deep_learning/detectnet "$@"
