set log x
set key right bottom
set xtics (20,50,100,200,500)
set ylabel "# evaluations"
set xlabel "LP dimension n  (dim(g)=5n)"

plot [10:800] \
'eval.txt' us 1:2:3 w errorbars ls 1 ps 0 lw 3 not,\
'eval.txt' us 1:2 w l ls 1 lw 3 t 'LogBarrier',\
'eval.txt' us 1:4:5 w errorbars ls 2 ps 0 lw 3 not,\
'eval.txt' us 1:4 w l ls 2 lw 3 t 'Aula',\
'eval.txt' us 1:6:7 w errorbars ls 3 ps 0 lw 3 not,\
'eval.txt' us 1:6 w l ls 3 lw 3 t 'AnyAula'

set terminal push
set terminal pdfcairo fontscale 0.7
set output 'z.pdf
replot
set terminal pop