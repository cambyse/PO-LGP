GIT-directory: mlr/src/share/test/


Tests have two purposes:
(1) Check whether our lib code has been compiled correctly and runs on your
    machine
(2) Learn how to use our lib code by means of demos
(Please note the difference to projects found in mlr/projects/.)



Naming-Convention:
All test-directories, which test something in src/DIR/FILE.h, are called
DIR_FILE or DIR_FILE_bla or DIR-file (to test some specific other file, maybe a
.cpp-file).





*** Many tests are self-explaining:
algos tests MT/algos.h
array tests MT/array.h
ors_* tests MT/ors.h
etc.




*** decisionMaking
Marc Toussaint, Tobias Lang
Tests the decision-making interface to the relational routines and the robot
maniplation simulator


*** earlyVision
Marc Toussaint
Tests the earlyVision module.


*** GP-IS-estimation
Stanimir Dragiev
Demonstrates the estimation of objects via Gaussian process implicit surfaces


*** grasp-ISF
Stanimir Dragiev
Demonstrates the feedback controller which is based on Gaussian process
implicit surfaces (GP-IS)


*** joystick_control
Marc Toussaint
Controlling Grobi with the joystick


*** perception
Marc Toussaint
Tests the perception module.


*** relational_basic
Tobias Lang
Tests basic relational routines.


*** relational_PRADA
Tobias Lang
Tests the planning algorithm PRADA.


*** relational_ruleLearning
Tobias Lang
Tests the relational rule learning engine.


*** schunk_hand
Marc Toussaint
Tests our SchunkHandModul in schunk.h.


*** schunk_skin
Marc Toussaint
Tests our SchunkSkinModul in schunk.h.


*** schunk_basicLibs
Marc Toussaint
Tests the Schunk library directly, without our wrappers.


*** vision_camera*
Nils Plath, Marc Toussaint
Tests Nils' camera drivers.
TODO should be renamed after refactoring (--> Michael, Andreas)


*** vision_colorBaseShapes, vision_opencv, vision_opencvOnline,
vision_segmentation
Marc Toussaint
Tests Marc's vision.h code.

