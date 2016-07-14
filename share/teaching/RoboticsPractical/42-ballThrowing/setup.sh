#!/bin/bash

# Create launch file for the right camera if it doesn't exist already
if [[ -e /opt/ros/indigo/share/ar_track_alvar/wristR.launch ]]
then
    echo "Skipping creation of wristR.launch..."
else
    echo "Creating file wristR.launch..."
    echo -e '<launch>\n    <arg name="marker_size" default="10.0" />\n    <arg name="max_new_marker_error" default="0.05" />\n    <arg name="max_track_error" default="0.1" />\n\n    <arg name="cam_image_topic" default="/cameras/right_hand_camera/image" />\n    <arg name="cam_info_topic" default="/cameras/right_hand_camera/camera_info" />\n    <arg name="output_frame" default="/reference/base" />\n\n    <node name="ar_track_alvar" pkg="ar_track_alvar" type="individualMarkersNoKinect" respawn="false" output="screen" args="$(arg marker_size) $(arg max_new_marker_error) $(arg max_track_error) $(arg cam_image_topic) $(arg cam_info_topic) $(arg output_frame)" />\n</launch>' >> /opt/ros/indigo/share/ar_track_alvar/wristR.launch
    echo "Done creating file wristR.launch..."
fi

sudo apt-get install python-dateutil\
    python-matplotlib\
    python-matplotlib-data\
    python-numpy\
    python-tk\
    python-scikits-learn\
    ipython\
    ros-indigo-visualization-msgs\
    ros-indigo-geometry-msgs
