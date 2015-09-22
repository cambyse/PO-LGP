Include = '../../../data/pr2_model/pr2_model.ors'

Include = '../../../data/pr2_model/pr2_modify_fixHead.ors'
Include = '../../../data/pr2_model/pr2_modify_fixGrippers.ors'
Include = '../../../data/pr2_model/pr2_modify_fixLeft.ors'

#Include = '../../../examples/Ors/ors_editor/regal.ors'

shape endeff(r_wrist_roll_link){ rel=<T t(.2 0 0)> type=5 color=[1 0 0] size=[.1 0 0 0]}

### tables

body table1{ type=0, X=<T t(1.5 1. .6) d(60 0 0 1)>, size=[1. 1. .04 .0], color=[.8 .5 .3] fixed, contact }
shape leg1(table1) { rel=<T t(-.3 -.3 -.3)>  type=0  size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }
shape leg2(table1) { rel=<T t(-.3 .3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }
shape leg3(table1) { rel=<T t(.3 .3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }
shape leg4(table1) { rel=<T t(.3 -.3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }

body table2{ type=0, X=<T t(1.5 -1. .6)>, size=[2. 2. .04 .0], color=[.8 .5 .3] fixed, contact }
shape leg1(table2) { rel=<T t(-.5 -.5 -.3)>  type=0  size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }
shape leg2(table2) { rel=<T t(-.5 .5 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }
shape leg3(table2) { rel=<T t(.5 .5 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }
shape leg4(table2) { rel=<T t(.5 -.5 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }

### tools

#Bodies are groups of shapes and come with their own transformations. It is also possible to include a shape already in the body definition (see tables).
#Transformations are 'turtle-like' - i.e. translations (t), rotations by degrees (d) or radians (r).
#Refer to center of shapes!
#QUESTION: Four components to rotation?
#Shapes are the building blocks of bodies and are the objects that are actually connected etc. in the geometric planner.
#The have another transformation relativ to their parent (transformations given in <>),
#a type (0 for box, 1 for sphere, then cappedCylinder, mesh, cylinder, ..),
#a size - four components x, y, z, radius. For boxes, set the latter 0 (gives some kind of rounded box), spheres ignore the first three, cylinders use last two.
#Contact means that a shape is considered for collisions - an underscore corrupts the keyword, thus unchecking the shape.
body tool1 { X=<T t(1.5 1. .64) d(240 0 0 1) t(0.0 -0.15 0.0)> }
shape bar1(tool1) { rel=<T t(.4 0 0)> type=0 size=[.8 .04 .04 0] color=[1 0 0] _contact }
shape hook1(tool1) { rel=<T t(.02 .15 0)> type=0 size=[.04 .3 .04 0] color=[1 0 0] _contact }

body tool2 { X=<T t(1.5 -1. .64) d(-140 0 0 1)> }
shape bar2(tool2) { rel=<T t(.2 0 0)> type=0 size=[.4 .04 .04 0] color=[1 0 0] _contact }
shape hook2(tool2) { rel=<T t(.02 .1 0)> type=0 size=[.04 .2 .04 0] color=[1 0 0] _contact }

body tool3 { X=<T t(1.5 -1 .6) t(0.1 -0.7 0.04) d(180 0 0 1)> }
shape bar3(tool3) { rel=<T t(.2 0 0)> type=0 size=[.4 .04 .04 0] color=[1 0 0] _contact }
shape hook3(tool3) { rel=<T t(.02 .1 0)> type=0 size=[.04 .2 .04 0] color=[1 0 0] _contact }

#Mystery collision?
body box1 { X=<T t(1.5 1. .6) d(60 0 0 1)> }
shape 1wall1(box1) { rel=<T t(0.0 0.25 0.3)> type=0 size=[1.2 0.04 0.3 0] color=[0 0 1.0] contact }
shape 1wall2(box1) { rel=<T t(0.0 -0.25 0.3)> type=0 size=[1.2 0.04 0.3 0] color=[0 0 1.0] contact }

body box2 { X=<T t(1.5 -1 .6) t(-0.3 -0.7 0) d(180 0 0 1)> }
shape 2wall1(box2) { rel=<T t(0.0 0.25 0.3)> type=0 size=[1 0.04 0.3 0] color=[0 0 1.0] _contact }
shape 2wall2(box2) { rel=<T t(0.0 -0.25 0.3)> type=0 size=[1 0.04 0.3 0] color=[0 0 1.0] _contact }


#shape hook(tool) { rel=<T t(.02 .2 0)> type=0 size=[.04 .04 .04 0] color=[1 1 0] contact }
#shape handle(tool) { rel=<T t(.5 0 0)> type=0 size=[.08 .04 .04 0] color=[1 1 0] }

#joint (graspRef tool){ B=<T t(-.5 0 0)> }
