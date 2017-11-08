body table { shape=9, X=<T t(0 0 .9)>, size=[1. .8 .04 .02], color=[.3 .3 .3] }

shape (table){ type=ST_box, rel=<T t(0 -.1 .02)>, size=[1. .013 .001 .0], color=[.1 .3 .1] }
shape (table){ type=ST_box, rel=<T t(0 +.15 .02)>, size=[1. .013 .001 .0], color=[.1 .3 .1] }

body yellow { type=9 size=[.05 .05 .1 .005] color=[.85 .85 0] contact }
body red    { type=9 size=[.05 .05 .1 .005] color=[.8 .1 .1] contact }
body blue   { type=9 size=[.05 .05 .10 .005] color=[.1 0 .6] contact }
body white  { type=9 size=[.05 .05 .1 .005] color=[1. 1. 1.] contact }
body black  { type=9 size=[.05 .05 .1 .005] color=[.1 .1 .1] contact }

joint (table yellow) { from=<T t(0  .15 .07) d(45 1 0 0) > type=JT_rigid }
joint (table red)    { from=<T t(0 -.07 .045) d(90 1 0 0) > type=JT_rigid }
joint (black blue)   { from=<T t(0 .05 0)> type=JT_rigid }
joint (red black)  { from=<T t(0 .05 0)> type=JT_rigid }
joint (table white)  { from=<T t(0  .075 .07)> type=JT_rigid }
