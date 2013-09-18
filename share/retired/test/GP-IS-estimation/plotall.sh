#!/bin/sh

plotcmd="
unset key
plot "
for seed in `seq 1 50`; do
  for size in `seq .2 .3  2`; do
    fn="exp1/$size-$seed"
    curdata="$(grep Obser $fn.log | cut -f3 -d: | cut -f1 -d' ')"
    [ -z "$curdata" ] && continue
    plotdata="$plotdata
$curdata
e"
    plotcmd="$plotcmd '-' with l lc rgb 'brown' , "
    echo $fn:
    echo $curdata
#    gnuplot -persist <<.
#    plot '-' with l
#$curdata
#.
# `read a`
  done
done

gnuplot -persist <<.
$plotcmd  0 with lines
$plotdata
.

