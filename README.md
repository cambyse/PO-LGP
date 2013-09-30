# MLR Repository

## Install on Ubuntu
Install the mlr system by executing:

  ./install/INSTALL

In "share/make-config" you can change the compile settings (un)set
dependencies.


## Install on Arch

What to do to install MLR on Arch Linux?

First install all dependencies. You need some things from AUR, so first install
yaourt or something similar to handle AUR packages.

    $ yaourt -S opencv plib ann lapack blas qhull ode

In order to get a working cblas library we need atlas-lapack, but be warned.
This literally will take hours of compile time (something like 5-10 h depending
on your system).

    $ yaourt -S atlas-lapack

(You can either drink a lot of coffee in the meantime or put LAPACK = 0 in your
make-config until it is finished...)

MLR needs a .so-file called libANN, ann provides one, but names it libann so we
link it:

    $ sudo ln -s /usr/lib/libANN.so /usr/lib/libann.so
    $ sudo ldconfig

qhull is newer in arch than in ubuntu and changes some pointer/reference things.
You can get the newer behaviour by adding

    QHULL2010 = 1

to your make-config.

Everything else is equal to ubuntu.


## Getting started

The currently only way to learn about this code and understand what is can do it to go through all examples.

Start with

 - examples/Core
 - examples/Gui
 - examples/Ors

All of them should work. Going through the main should be instructive on how you could use the features. In examples/Ors some demos depend on linking to Physx which is turned off on default.

Continue with more specialized packages:

 - examples/Optim
 - examples/Motion
 - examples/Algo
 - examples/Infer

which implement algorithmic methods.

Finally examples/Hardware contains drivers/recorders for Kinect, Cams, G4.

examples/System is code for designing parallized modular systems (using Core/module.h) - which I think works robustly but needs more tools for others to be usable.

examples/relational is Tobias relational RL code


## Documentation

Create the doxygen documentation with:

    cd share
    make doc
