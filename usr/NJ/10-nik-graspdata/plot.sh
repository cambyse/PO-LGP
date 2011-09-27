#!/usr/bin/env sh 
#
# This script is tightly coupled with the plot_append_data() function.
# Namely, it relies on the date being appended as it is done there.
#
# Be sure to reflect changes made here to appropriate changes in the
# plot_append_data() function and plot_all() functions of the binary.

gnuplot -persist - <<EOF
set title "ISF value at eef"; \
set xlabel "time step"; \
set ylabel "ISF value"; \
plot \
'z.plotdata' every :::0::0 with l title "palm(x(t))",\
'z.plotdata' every :::1::1 with l title "fg1(x(t))",\
'z.plotdata' every :::2::2 with l title "tip1(x(t))",\
'z.plotdata' every :::8::8 with l title "fg2(x(t))",\
'z.plotdata' every :::9::9 with l title "tip2(x(t))",\
'z.plotdata' every :::15::15 with l title "fg3(x(t))",\
'z.plotdata' every :::16::16 with l title "tip3(x(t))"\
;
EOF

gnuplot -persist - <<EOF
set title "orientation-TVs' error weighted by precision"; \
set xlabel "time step"; \
set ylabel "prec*(target-cur)"; \
plot \
'z.plotdata' every :::3::3 with l title "loss(fg1)",\
'z.plotdata' every :::4::4 with l title "loss(tip1)",\
'z.plotdata' every :::10::10 with l title "loss(fg2)",\
'z.plotdata' every :::11::11 with l title "loss(tip2)",\
'z.plotdata' every :::17::17 with l title "loss(fg3)",\
'z.plotdata' every :::18::18 with l title "loss(tip3)",\
'z.plotdata' every :::22::22 with l title "loss(skin)"\
;
EOF

gnuplot -persist - <<EOF
set title "precision of orientation TVs"; \
set xlabel "time step"; \
set ylabel "precision"; \
plot \
'z.plotdata' every :::6::6 with l title "prec(fg1)",\
'z.plotdata' every :::7::7 with l title "prec(tip1)",\
'z.plotdata' every :::13::13 with l title "prec(fg2)",\
'z.plotdata' every :::14::14 with l title "prec(tip2)",\
'z.plotdata' every :::20::20 with l title "prec(fg3)",\
'z.plotdata' every :::21::21 with l title "prec(tip3)"\
;
EOF

gnuplot -persist - <<EOF
set title "Nominal error of skin var"; \
set xlabel "time step"; \
set ylabel "nom err"; \
plot \
'z.plotdata' every :::5::5 with l title "err(skin1)",\
'z.plotdata' every :::12::12 with l title "err(skin2)",\
'z.plotdata' every :::19::19 with l title "err(skin3)"\
;
EOF

