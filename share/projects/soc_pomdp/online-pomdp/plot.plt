set out "appolo.ps" # an output postscript file
set terminal postscript
set key font ",30"
set key spacing 5
set key samplen 4
set xtics font ",25"
set ytics font ",25"
set xlabel font ",25"
set ylabel font ",25"
set ylabel ""
set xlabel ""

set key bottom right

set yrange [0.2:1.2]
set xrange [0:300]

set grid x y


plot "data-1.dat" using 1:3 title 'z_table' with lines linewidth 4 linecolor 1,\
	"data-1.dat" using 1:4 title 'z_eff' with lines linewidth 4 linecolor 2,\
    "data-1.dat" using 1:7 title 'z_target' with lines linewidth 4 linecolor 3,\
    "data-1.dat" using 1:6 title 'z_est' with lines linewidth 4 linecolor 4


