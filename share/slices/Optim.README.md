# The Optimization slice of the MLR code base

This code is a slice of the full MLR code base, containing
optimization methods.

## Installation

Generically the following should work:

cd share
make
cd examples/Optim/optimization
make
./x.exe

On problems, maybe you have to install some more Ubuntu
packages or adapt to other OS. Please refer to the files

* README.md
* install/INSTALL_ALL_UBUNTU_PACKAGES.sh

These two refer to the whole MLR code base. You
probably need to install only a fraction of these packages.

## Examples

In share/examples/Optim there are currently three examples:

* 'optimization' is a minimalistic test
* 'constrained' tests the constrained methods
* 'kOrderMarkov' tests the application of constrained optimization on
  cost/constraint functions that are of a k-order-Markov form. This is
  a very general form of function that includes robot trajectory
  optimization problems. The trick is that these functions lead to
  banded Hessians, which can be inverted efficiently in the Newton steps.

Contact me:

Marc Toussaint
marc.toussaint@informatik.uni-stuttgart.de



