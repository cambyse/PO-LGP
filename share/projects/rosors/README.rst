======
RosOrs
======

RosOrs is supposed to be a general bridge between ros and ors.  This is kinda
a subproject of TCR but can be and should be used by everybody who works with
ors and ros.

RosOrs should be an ROS package.  One should be able to add the RosOrs
dependency to ones project and use (most of) the ors functionality.


Dependencies
===============

ROS Dependencies:
 - ors_msgs

MLR Dependencies:
 - orspy
 - corepy
 - guipy

Install
=======
::

    ln -s ~/git/mlr/share/projects/rosors/ ~/fuerte_workspace/sandbox/rosors
    source ~/fuerte_workspace/setup.sh
    rosmake rosors


Getting Started
===============
Run rosors.py in scripts/::

    python rosors.py


Log / Journal
===============

2013-11-14 Thu
-----------------

 - DONE Turn RosOrs into a full ROS dependency.
   What should be in scripts/, src/ etc?
   See resources on ros packages: http://wiki.ros.org/rosbuild/Packages
   More precisely::

     - bin/: compiled binaries
     - include/package_name: C++ include headers (make sure to export in the Manifest)
     - msg/: Message (msg) types
     - src/package_name/: Source files, especially Python source that are exported to other packages.
     - srv/: Service (srv) types
     - scripts/: executable scripts
     - CMakeLists.txt: CMake build file (see CMakeLists)
     - manifest.xml: Package Manifest
     - mainpage.dox: many packages will often place their Doxygen mainpage documentation here

 - DONE Add example of rosors.

 - DONE Extract the msgs into a sep. ros package. see ../ors_msgs

 - TODO save the following
   - body type
   - shape type
   - mesh
