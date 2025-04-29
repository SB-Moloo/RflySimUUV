#!/bin/bash

dir1="/home/rflysim/uuv20240309"

gnome-terminal --window \
--tab --working-directory=${dir1} --title="ros" --command="bash -c 'roscore; exec bash'" \
--tab --working-directory=${dir1} --title="rviz" --command="bash -c 'sleep 5s; source ~/ros1_ws/devel/setup.bash; roslaunch vins vins_rviz.launch; exec bash'" \
--tab --working-directory=${dir1} --title="vins" --command="bash -c 'sleep 5s; source ~/ros1_ws/devel/setup.bash; rosrun vins vins_node ~/ros1_ws/src/VINS-Fusion/config/euroc_uuv/rflysim_stereo_imu_config.yaml; exec bash'" \

sleep 10s

gnome-terminal --window \
--tab --working-directory=${dir1} --title="rope" --command="bash -c 'sleep 1s; python3 ropeInfo_generator.py; exec bash'" \
--tab --working-directory=${dir1} --title="relocate" --command="bash -c 'sleep 2s; python3 relocate.py; exec bash'" \
--tab --working-directory=${dir1} --title="fusion" --command="bash -c 'sleep 3s; python3 path_fusion.py; exec bash'" \
--tab --working-directory=${dir1} --title="control" --command="bash -c 'sleep 4s; python3 UUVAtt_server.py; exec bash'" \

