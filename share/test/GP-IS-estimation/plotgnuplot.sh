#!/bin/sh

# plot median and lower and upper 20-quantiles as computed by plot1.r


## make a custom plot in pdf via eps.
# Can set fonts and stuf...
# example:
#  $create_pdf_plot "GP_learning_ISF_a" "set grid; set ytics 1; load \"z.plotcmd\" "font 35 dl 5 lw 8 solid"
###
create_pdf_plot () {
name=$1
content=$2
eps=$name.eps
pdf=$name.pdf
psoptions=$3

echo plotting to $eps...
gnuplot > $eps <<EOF
set term postscript eps color enhanced $psoptions 
$content
EOF

echo converting to $pdf...
epstopdf $eps --outfile="$pdf"
echo done

}

plotcmd="\
plot \
 '-'  using 1:3:4 with filledcurves fill solid 0.4 lc rgb 'yellow' notitle, \
 '-'  using 1:2:3 with filledcurves fill solid 0.4 lc rgb 'yellow' notitle, \
 '-'  using 1:4 with l lc rgb 'gray' title '.95' , \
 '-'  using 1:3 with l lc rgb 'green' title '.5 ' , \
 '-'  using 1:2 with l lc rgb 'brown' title '.05' \
"
plotdata="`R --vanilla <plot1.r | grep '^[0-9]'`"

cmd="
set grid
set key bottom right
set xtics 10
set ytics .2
$plotcmd
$plotdata
e
$plotdata
e
$plotdata
e
$plotdata
e
$plotdata
"

create_pdf_plot "GPISF-sanity" "$cmd" "font 35 dl 5 lw 8 solid"

##cat <<.
#gnuplot  -persist <<.
#$cmd
#.
