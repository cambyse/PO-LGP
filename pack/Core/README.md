# Core package of the Machine Learning & Robotics code base

(Linear Algebra, Array, Graph, Thread)

This code is a slice of the full MLR code base, containing
optimization methods. This is also the reason for the unusual
directory layout.


## Installation

Generically, the following should work:

cd share
make
cd test/Core/array
make
./x.exe

To run examples anywhere:
export LD_LIBRARY_PATH="$HOME/git/mlr/share/src"

Next step: try everything in test/ and examples/

On problems, maybe you have to install some more Ubuntu packages. In
build/depend-dep all needed packages are listed. See below for more
tips.


## Content

The MLR-Core package contains basic data structures that are used
throughout the MLR code and a few utilities. The data structures are:

### Array

A generic array container class. Objectives: simplicity, conciseness
of notation, similarity to Matlab (overloaded functions and
operators to match matlab), efficient calling of LAPACK routines,
efficient interfacing with external libraries that use othere array
data structures (Eigen, std::vector, OpenCV, etc).

### Graph

A quite powerful any-thing container class: One could call it an
'any-type key-value hierarchical hypergraph', meaning: 1) Every node
can store any type, with functionalities to access types run-time, and
clone such types run-time. Using only this functionality, the data
structure is an any-type list. 2) Every nodes has string keys. Using
this functionality, the container is like a std::map or key-value list
(compatible to Phyon dictionaries). 3) Every node can have other nodes
as parents and thereby becomes an hyperedge. This renders the data
structure a graph, with compatibility to graphviz (dot). 4) Every node
can be of type Graph, meaning that this node is a subgraph; thereby
the data structure becomes hierarchical, similar to how XML is
hierarchical (a typical example for a subgraph is a attribute list of
some node).

We use the data structure for all kinds of things internally, similar
to Phython dictionarites. But more importantly, 1) to represent a
logic knowledgebase, 2) to represent a kinematic world, 3) to register
currently running threads in the process (in more involved
applications), 4) to parse configuration/parameter/kinematics/anything
files coherently with into the same data structure.

### Thread and Access

A minimalistic pattern to run threads. This does not aim at a general
threading, but enforce a concrete pattern: Threads should ONLY
implement open/step/close; stepping is done with a fixed frequency or
triggered from other threads writing into a mutex-protected memory;
threads should only ever access (read/write) mutex-protected memory
variables. This is supported via that 'Access' class, which creates
such variables and controls/logs the accesses. Implemented with basic
pthread (optionally also with QtThreads, but that doesn't change a thing).

We use the Thread/Access pattern throughout when it gets to building
systems NEXT TO using ROS. The Thread/Access structures internal
processes. Typical example threads are: the higher control loop
(100Hz), drivers to read from sensors (gamepad, camera), display loops
with opengl. OpenGL (in the MLR-Gui) is made thread-safe.

### Utils

Not necessarily to be used outside. Perhaps the String and FileToken
classes are useful, as they allow easier piping of variables in all
four possible ways (var >>string; var <<string; string >>var;
string <<var; same for a file)

### binaries, extras, external dependencies

No binaries.

This package contains generic makefile defines (generic.mk and
defines.mk) and a make-path.sh script, that are used by our
old-fashioned makefile setup (to compile across packages including
dependencies).

The package depends only on lapack, f2c, pthread



## Tips beyond just the Core package

### Install on Arch

What to do to install MLR on Arch Linux?

First install all dependencies. You need some things from AUR, so first install
yaourt or something similar to handle AUR packages.

    $ yaourt -S opencv plib ann lapack blas qhull

In order to get a working cblas library we need atlas-lapack, but be warned.
This literally will take hours of compile time (something like 5-10 h depending
on your system).

    $ yaourt -S atlas-lapack

(You can either drink a lot of coffee in the meantime or put LAPACK = 0 in your
gofMake/config.mk until it is finished...)

MLR needs a .so-file called libANN, ann provides one, but names it libann so we
link it:

    $ sudo ln -s /usr/lib/libANN.so /usr/lib/libann.so
    $ sudo ldconfig

qhull is newer in arch than in ubuntu and changes some pointer/reference things.
You need to put 

   ARCH_LINUX = 1

in your gofMake/config.mk. This fixes the qhull change.

Everything else is equal to ubuntu.


### Install troubleshooting

* If there is a segmentation fault in c_init (before main) in some
  boost library, maybe this is caused by linking to PCL in the
  src/Perception code. Fix: in share/gofMake/config.mk insert a line
  'PCL=0', which disables linking to PCL.


### QtCreator tips

* Enable Debugging helpers:
  .gdbinit -> git/mlr/tools/qt_mlr_types.py

* Enable std::... pretty printing:
  Options > Debugger > GDB ... UNCHECK the 'Load system GDB pretty printers'

* set formatting:
  Options > C++ > Import...   git/mlr/tools/qt_coding_style.xml
  
* Change signal handling of gdb
  Open 'Debugger Log' window
  write gdb commands:
  handle SIGINT pass
  handle SIGINT noprint
  handle SIGINT nostop
  handle SIGINT ignore

* append source directory to all "*.includes" files:
echo "echo '/home/mtoussai/git/mlr/share/src' >> \$1" > nogit_append_path
chmod a+x nogit_append_path
find . -name '\.*\.includes' -exec ./nogit_append_path {} \;

### Windows

Large parts of the code compile with MSVC. Try this:

download https://ipvs.informatik.uni-stuttgart.de/mlr/marc/source-code/libRoboticsCourse.13b.msvc.tgz

goto  http://www.microsoft.com/en-us/download/developer-tools.aspx

search for Visual C++ 2010 Express

download and install

open the 'solution' msvc/RoboticsCourse.sln

Creating a new project is tricky! You could avoid it by copying new
code into an existing project, replacing the main.cpp (e.g. in
example\Ors\ors). But in case you need to:

When creating new projects:
-- set the MT_MSVC preprocessor flag (perhaps also MT_FREEGLUT,
   MT_ANN, MT_GL, MT_QHULL)
-- set the Debugger work directory to ..\..\share\example\Ors\ors or
   alike
-- set an Additional Includepath to ..\..\share\src
-- perhaps set the VC++ Directory 'libpath' to ..\..\msvc\freeglut\lib
-- add the created lib-files in msvc\Debug to the project (drag and
   shift-drop from others)
-- understand, that programs are by default run in msvc\Debug
-- perhaps you need to copy dlls around - (don't know how to set the
   path globally)


### Run on Virtual Machines

Setting up this software to run on a virtual machine is an option for whoever
can't make it work on his OS. The biggest issue of this approach is getting the
OpenGL rendering to work.

Instructions:

1) Install the most recent version of virtualbox for your OS.  (don't use the
package repository, use https://www.virtualbox.org/wiki/Downloads).

2) Download the ISO for the desktop-amd64 version of Ubuntu (12.04 is
recommended, more recent versions may also work) and install it on a new VM.

3) run:
sudo apt-get update
sudo apt-get upgrade

4) [Optional] If the above software updates are slow, close the VM and change
the network setting "Attached to: Bridged Adapter".

5) [Optional] Step 6 is said to require the linux-headers of the current
version.  Nonetheless, I've made everything work already before without this
step.

run:
sudo apt-get install build-essential linux-headers-$(uname -r)

6) Select Devices -> Insert Guest Additions CD Image. Then run the autorun
script in the mounted CD drive. This installs the virtualbox-guest-addition
software specific for your virtualbox version (don't use package repository
version).  Check that everything is installed correctly.

7) If there were no problems with the installation, unmount the CD drive and
reboot the VM.

8) Proceed installing the MLR code base.

Tips)

Take frequent snapshots of the VM, whenever you're in a state where it is
working correctly (this also helps during the above setup procedure). If things
break at some point, transfer the code you've written (either using some
versioning tool, or a USB stick) to the most recent snapshot of the VM and
proceed working.




## Contact:

Marc Toussaint
marc.toussaint@informatik.uni-stuttgart.de



