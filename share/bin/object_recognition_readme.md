Packages to install:

ros-indigo-freenect-launch  <--- you should also be able to
alternatively use openni, but I was able to get this working faster
ros-indigo-object-recognition-ros
ros-indigo-object-recognition-msgs
ros-indigo-object-recognition-ros-visualization
ros-indigo-object-recognition-tabletop
ros-indigo-object-recognition-core


1) First, start up the kinect:

roslaunch freenect_launch freenect-registered-xyzrgb.launch data_skip:=2

2) To run the object recognition tabletop example:
rosrun object_recognition_core detection -c `rospack find
object_recognition_tabletop`/conf/detection.table.ros.ork

Please note that this is different from the detection.table.ros.ork
found in mlr/rospath/, because the topics have different names.

3) Once these are both up and running:

rosrun rviz rviz

4) In the global settings, change the fixed frame to be
"camera_depth_optical_frame".

5) Add a Pointcloud2 display, and change the topic to:
/camera/depth_registered/points
6) Add an OrkTable as display, and change the topic to /table_array

You should now see a point cloud with planes extracted.

-------------------

If you are able to see the point cloud, but only getting a few frames
per second, please do the following:

rosrun rqt_reconfigure rqt_reconfigure

Change the Camera/Driver/data_skip parameter to a higher number, and
you should see an increase frame rate.


