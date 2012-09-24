#!/bin/sh


gnuplot<<EOF
set terminal svg
set out "data-$1.svg"

set view 75,30,1,1

set xyplane at -.6

set xrange [-1.5:1.5]
set yrange [-1.5:1.5]
set zrange [-.5:.5]

set grid

#splot 'true-$1.data' t "On (correct recognized)", 'false-$1.data' t "Not on (correct recognized)", 'fp-$1.data' t "False positives",'fn-$1.data' t "False negatives"
splot 'true-$1.data' t "On (correct recognized)", 'fp-$1.data' t "False positives",'fn-$1.data' t "False negatives"

unset out
set out "data-rand-$1.svg"
splot 'true-rand-$1.data' t "On (correct recognized)", 'fp-rand-$1.data' t "False positives",'fn-rand-$1.data' t "False negatives"
EOF
