body table1{ type=9, X=<T t(0 0 .7)>, size=[2. 3. .04 .02], color=[.3 .3 .3] fixed, contact }

### ball

body block{ type=ST_ssBox, X=<T t(0 0 .8)>, size=[.1 .1 .3 .02], color=[.9 .3 .3] fixed, contact }
joint (table1 block){ type=0 rel=<T t(0 0 .5)> }


