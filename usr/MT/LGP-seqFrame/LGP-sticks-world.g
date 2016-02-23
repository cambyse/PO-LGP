
body table{ type=9, X=<T t(2. 0 .6) d(90 0 0 1) >, size=[2. 1.6 .02 .02], color=[.8 .5 .3] contact }
shape leg1(table) { rel=<T t(-.3 -.3 -.3)>  type=0  size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }
shape leg2(table) { rel=<T t(-.3 .3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }
shape leg3(table) { rel=<T t(.3 .3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }
shape leg4(table) { rel=<T t(.3 -.3 -.3)>  type=0 size=[.04 .04 .6 0] color=[0.5 0.3 0.15] contact }

#Include='floatingArm.ors'
Include='pr2.ors'

body obstacles{ X=<T t(2. 0 .6) d(90 0 0 1) > }
shape (obstacles) { type=9 rel=<T t(-1 -.0 .15)> size=[.02 1.6 .3 .02] color=[.8 .5 .3] contact }
shape (obstacles) { type=9 rel=<T t(-.5 -.2 .15)> size=[.02 1.2 .3 .02] color=[.8 .5 .3] contact }
shape (obstacles) { type=9 rel=<T t(0 -.2 .15)> size=[.02 1.2 .3 .02] color=[.8 .5 .3] contact }
shape (obstacles) { type=9 rel=<T t(.5 -.2 .15)> size=[.02 1.2 .3 .02] color=[.8 .5 .3] contact }
shape (obstacles) { type=9 rel=<T t(1 -.0 .15)> size=[.02 1.6 .3 .02] color=[.8 .5 .3] contact }
shape (obstacles) { type=9 rel=<T t(0 -.8 .15)> size=[2. .02 .3 .02] color=[.8 .5 .3] contact }

body stick1 {}
joint (table stick1) { from=<T t(-.75 .5 .06)> type=10 }
shape stick1(stick1) { type=9 size=[.02 .3 .02 .02] color=[1 0 0] contact }
shape hook1(stick1) { type=9 rel=<T t(0 .15 0)> size=[.2 .02 .02 .02] color=[1 0 0] contact }
body stick2 {}
joint (table stick2) { from=<T t(-.25  .1 .06)> type=10 }
shape stick2(stick2) { type=9 size=[.02 .5 .02 .02] color=[1 0 0] contact }
shape hook2(stick2) { type=9 rel=<T t(0 .25 0)> size=[.2 .02 .02 .02] color=[1 0 0] contact }
body stick3 {}
joint (table stick3) { from=<T t( .25  -.3 .06)> type=10 }
shape stick3(stick3) { type=9 size=[.02 .7 .02 .02] color=[1 0 0] contact }
shape hook3(stick3) { type=9 rel=<T t(0 .35 0)> size=[.2 .02 .02 .02] color=[1 0 0] contact }
%body stick4 {}
%joint (table stick4) { from=<T t( .75 -.25 .06)> type=10 }
%shape (stick4) { type=9 size=[.02 .9 .02 .02] color=[1 0 0] contact }
%shape (stick4) { type=9 rel=<T t(0 .45 0)> size=[.2 .02 .02 .02] color=[1 0 0] contact }

body target {}
joint (table target) { from=<T t( .75 -.5 .08)> type=10 }
shape target(target) { type=9 size=[.06 .06 .06 .02] color=[1 0 0] contact }


