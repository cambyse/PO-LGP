=====================
pr2_simple_trajectory
=====================

This demo moves the arm of the PR2.

This is mostly taken from:
http://wiki.ros.org/pr2_controllers/Tutorials/Moving%20the%20arm%20using%20the%20Joint%20Trajectory%20Action

Install
=====================

Link from the mlr repo to the fuerte_workspace::

    ln -s ~/git/mlr/share/projects/pr2_simple_trajectory ~/fuerte_workspace/

Build the project::

    rosmake pr2_simple_trajectory

Run the executable::

    roscd pr2_simple_trajectory
    cd bin
    ./pr2_simple_trajectory
