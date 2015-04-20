# The Optimization slice of the MLR code base

This code is a slice of the full MLR code base, containing
basic ML methods. This is also the reason for the unusual
directory layout.


## Installation

Generically, the following should work:

cd share
make
cd examples/Core/array
make
./x.exe

On problems, first try to disable LAPACK by inserting LAPACK=0 in
gofMake/config.mk. Further, maybe you have to install some more Ubuntu
packages or adapt to other OS. Please refer to the files

* README.mlr.md
* install/INSTALL_ALL_UBUNTU_PACKAGES.sh

These two refer to the whole MLR code base. You probably need to
install only a fraction of these packages.


## Contact me:

Marc Toussaint
marc.toussaint@informatik.uni-stuttgart.de



