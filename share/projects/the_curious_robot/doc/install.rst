Getting Started
=====================
TCR uses **ROS Fuerte** and the **MLR** code. This section explains how to setup everything.

Prerequisites
-------------
This section explains how to

- setup ROS,
- setup `articulation` for ROS,
- activate PhysX
- compile the python wrappers

ROS
~~~~~~~~~~~~~
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


Articulation
~~~~~~~~~~~~~
- install the ros articulation package by Sturm in the sandbox::

    cd ~/fuerte_workspace/sandbox
    rosws set alufr-ros-pkg --svn http://alufr-ros-pkg.googlecode.com/svn/trunk/
    rosws update

- compile articulation::

    rosmake articulation

MLR: PhysX
~~~~~~~~~~~~~~~~~~~
TCR depends on the python bindings of MLR and on PhysX support.

To install PhysX call the install script in the install folder::

    install/PhysX_install

Then activate the PhysX support by addind the following line to
share/make-config::

    PHYSX = 1

and recompile the entire project::

    make clean
    make

MLR: Python Wrapper
~~~~~~~~~~~~~~~~~~~
Also compile the python wrappers::

    cd share/src/Core
    make -f Makefile_corepy
    cd ../Ors
    make -f Makefile_orspy


Setup & Compile TCR
-------------------

- create a symlink from TCR project ``~/git/mlr/share/projects/the_curious_robot/`` to
  ``~/fuerte_workspace/sandbox/the_curious_robot``::

    ln -s ~/git/mlr/share/projects/the_curious_robot/ ~/fuerte_workspace/sandbox/the_curious_robot

- call cmake and in ``~/fuerte_workspace/sandbox/the_curious_robot/`` to compile
  the project::

    cmake .
    make
    rosmake the_curious_robot

- We need an `MT.cfg` for the RRT of TCR in `~/.ros`:: 

      ln -s ~/fuerte_workspace/sandbox/the_curious_robot/src/MT.cfg ~/.ros/MT.cfg

Run
----

Run TCR with::

    cd ~/fuerte_workspace/sandbox/the_curious_robot/src
    # start TCR
    roslauch the_curious_robot.launch
    # display the behavior state machine
    rosrun smach_viewer smach_viewer.py
