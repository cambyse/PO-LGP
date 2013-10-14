Getting Started
=====================
TCR uses **ROS Fuerte** and the **MLR** code. This section explains how to setup everything.


Prerequisites
-------------

This explains how to setup ROS and articulation which is used by TCR.

- use Ubuntu 12.04 and ROS Fuerte!
- install libgsl0-dev to make it work::

    sudo apt-get install libgsl0-dev

- install ros fuerte and additional packages::

    sudo apt-get install ros-fuerte-desktop-full ros-fuerte-rqt

  see http://www.ros.org/wiki/fuerte/Installation/Ubuntu
- create a ros fuerte workspace in ``~/fuerte_workspace`` and a sandbox directory
  ``~/fuerte_workspace/sandbox``
  see http://www.ros.org/wiki/ROS/Tutorials/InstallingandConfiguringROSEnvironment ::

    rosws init ~/fuerte_workspace /opt/ros/fuerte
    mkdir ~/fuerte_workspace/sandbox
    rosws set ~/fuerte_workspace/sandbox
    source ~/fuerte_workspace/setup.bash

- install the ros articulation package by Sturm in the sandbox::

    cd ~/fuerte_workspace/sandbox
    rosws set alufr-ros-pkg --svn http://alufr-ros-pkg.googlecode.com/svn/trunk/
    rosws update

- compile articulation::

    rosmake articulation


Setup
--------
- create a symlink from TCR project ``~/git/mlr/share/projects/the_curious_robot/`` to
  ``~/fuerte_workspace/sandbox/the_curious_robot``::

    ln -s ~/git/mlr/share/projects/the_curious_robot/ ~/fuerte_workspace/sandbox/the_curious_robot

- call cmake and in ``~/fuerte_workspace/sandbox/the_curious_robot/`` to compile
  the project::

    cmake .
    make
    rosmake the_curious_robot


Run
----

Run TCR with::

    cd ~/fuerte_workspace/sandbox/the_curious_robot/src
    # start TCR
    roslauch the_curious_robot.launch
    # display the behavior state machine
    rosrun smach_viewer smach_viewer.py
