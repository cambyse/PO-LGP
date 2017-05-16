# Quick Start on Ubuntu 16.04
Install git:

    $ sudo apt-get install git

Add your ssh key to clone the repository.

    User->Settings->SSH Keys

Clone the repo:
    
    mkdir ~/git/; cd ~/git/
    git clone git@animal.informatik.uni-stuttgart.de:mlr-staff/mlr.git
    
Once the repo has been cloned:
    cd mlr
    git checkout roopi
    ./install/INSTALL_ALL_UBUNTU_PACKAGES.sh
    ./install/INSTALL_ROS_PACKAGES_KINETIC
    ./share/bin/createMakefileLinks.sh
    cd share/examples/Roopi/basics/
    make
