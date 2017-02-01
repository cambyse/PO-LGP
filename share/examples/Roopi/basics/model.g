#Include = 'arm7.kvg'
Include = '../../../data/pr2_model/pr2_model.ors'

body table{ type=0, X=<T t(0 -1.2 .6)>, size=[1. 1. .04 .0], color=[.8 .5 .3] fixed,  }
shape leg1(table) { rel=<T t(-.3 -.3 -.3)>  type=0  size=[.04 .04 .6 0] color=[0.5 0.3 0.15] }
shape leg2(table) { rel=<T t(-.3 .3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] }
shape leg3(table) { rel=<T t(.3 .3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] }
shape leg4(table) { rel=<T t(.3 -.3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] }

body table2{ type=0, X=<T t(1.2 0 .6)>, size=[1. 1. .04 .0], color=[.8 .5 .3] fixed,  }
shape leg1(table2) { rel=<T t(-.3 -.3 -.3)>  type=0  size=[.04 .04 .6 0] color=[0.5 0.3 0.15] }
shape leg2(table2) { rel=<T t(-.3 .3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] }
shape leg3(table2) { rel=<T t(.3 .3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] }
shape leg4(table2) { rel=<T t(.3 -.3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] }

body slider { type=ST_box size=[.2 .1 .05 0] color=[0 0 0] }
joint sliderJoint(table slider){ type=JT_hingeZ  ctrl_H=1 }

body obj1 { size=[.1 .1 .2 .02] type=9 contact }
#body obj2 { size=[.1 .1 .2 0] type=0 contact }
#body obj3 { size=[.1 .1 .2 0] type=0 contact }
#body obj4 { size=[.1 .1 .2 0] type=0 contact }
#body obj5 { size=[.1 .1 .2 0] type=0 contact }
#body obj6 { size=[.1 .1 .2 0] type=0 contact }

#joint (slider obj1){ type=JT_transX }
joint (table obj1) { from=<T t(0 0 .02)> to=<T t(0 0 .1)> type=JT_rigid}

#joint (table obj2) { from=<T t(-.2 .1 0)> to=<T t(0 0 .1)> type=10 }
#joint (table obj3) { from=<T t(-.3 -.1 0)> to=<T t(0 0 .1)> type=10 }
#joint (table obj4) { from=<T t(-.1 -.1 0)> to=<T t(0 0 .1)> type=10 }
#joint (table obj5) { from=<T t(-.1 -.2 0)> to=<T t(0 0 .1)> type=10 }
#joint (table obj6) { from=<T t(-.1 .3 0)> to=<T t(0 0 .1)> type=10 }

shape target(table2){ type=5 rel=<T t(0 0 .15)> size=[.3 0 0 0] color=[1 1 0]}



#shape endeffL(l_wrist_roll_link){ rel=<T t(.2 0 0)> type=5 color=[1 0 0] size=[.1 0 0 0]}
#shape endeffR(r_wrist_roll_link){ rel=<T t(.2 0 0)> type=5 color=[1 0 0] size=[.1 0 0 0]}
