#set termopt enhanced
#set macros

data_folder='../evaluations/2013_10_ICML/'

rn(x) = 0.6*x*invnorm(rand(0))

## 2x2
PS=1.5; \
set size ratio 0.5; \
set key bottom right Left; \
set format y "%.2f"; \
set ytics 0.05; \
set mxtics 2; \
set mytics 5; \
set xlabel 'Length of Training Episode'; \
set ylabel 'Mean Reward over 12 steps'
plot [:][:] \
     data_folder.'2x2_SPARSE_plot.txt' u 1:2 w lp pt 6 ps PS lt 1 lw 2 t 'CRF', \
     data_folder.'2x2_UTREE_VALUE_plot.txt' u 1:2 w lp pt 6 ps PS lt 2 lw 2 t 'U-Tree (value-based)', \
     data_folder.'2x2_UTREE_PROB_plot.txt' u 1:2 w lp pt 6 ps PS lt 3 lw 2 t 'U-Tree (model-based)' , \
     data_folder.'2x2_LINEAR_Q_plot.txt' u 1:2 w lp pt 6 ps PS lt 7 lw 2 t 'Linear-Q'
unset mytics; \
set ytics auto; \
unset format;
plot [:][:] \
     data_folder.'2x2_SPARSE_plot.txt' u 1:3 w lp pt 6 ps PS lt 1 lw 2 t 'CRF', \
     data_folder.'2x2_UTREE_VALUE_plot.txt' u 1:3 w lp pt 6 ps PS lt 2 lw 2 t 'U-Tree (value-based)', \
     data_folder.'2x2_UTREE_PROB_plot.txt' u 1:3 w lp pt 6 ps PS lt 3 lw 2 t 'U-Tree (model-based)' , \
     data_folder.'2x2_LINEAR_Q_plot.txt' u 1:3 w lp pt 6 ps PS lt 7 lw 2 t 'Linear-Q'
plot [:][:] data_folder.'2x2_SPARSE.txt' u ($2+rn(10)):($4+rn(0.01)) w p pt 0 ps 1 lt 1 lw 2 t 'CRF'
plot [:][:] data_folder.'2x2_UTREE_VALUE.txt' u ($2+rn(10)):($4+rn(0.01)) w p pt 0 ps 1 lt 2 lw 2 t 'U-Tree (value-based)'
plot [:][:] data_folder.'2x2_UTREE_PROB.txt' u ($2+rn(10)):($4+rn(0.01)) w p pt 0 ps 1 lt 3 lw 2 t 'U-Tree (model-based)'
plot [:][:] data_folder.'2x2_LINEAR_Q.txt' u ($2+rn(10)):($4+rn(0.01)) w p pt 0 ps 1 lt 7 lw 2 t 'Linear-Q'

## 4x4
PS=1; \
PT=0; \
set size ratio 0.5; \
set key bottom right Left; \
set format y "%.2f"; \
set ytics 0.05; \
set mxtics 2; \
set mytics 5; \
set xlabel 'Length of Training Episode'; \
set ylabel 'Mean Reward over 22 steps';
plot [0:5100][:] \
     data_folder.'4x4_III_SPARSE_plot.txt' u 1:2:3 w errorbars pt PT ps PS lt 1 lw 2 t 'CRF', \
     data_folder.'4x4_III_UTREE_VALUE_plot.txt' u 1:2:3 w errorbars pt PT ps PS lt 2 lw 2 t 'U-Tree', \
     data_folder.'4x4_III_SPARSE_plot.txt' u 1:2 w l lt 1 lw 2 t '', \
     data_folder.'4x4_III_UTREE_VALUE_plot.txt' u 1:2 w l lt 2 lw 2 t ''
unset mytics; \
set ytics auto; \
unset format;
plot [:][:] \
     data_folder.'4x4_III_SPARSE_plot.txt' u 1:3 w lp pt PT ps PS lt 1 lw 2 t 'CRF', \
     data_folder.'4x4_III_UTREE_VALUE_plot.txt' u 1:3 w lp pt PT ps PS lt 2 lw 2 t 'U-Tree (value-based)'
plot [:][:] data_folder.'4x4_III_SPARSE.txt' u ($2+rn(10)):($4+rn(0.01)) w p pt 0 ps 1 lt 1 lw 2 t 'CRF'
plot [:][:] data_folder.'4x4_III_UTREE_VALUE.txt' u ($2+rn(10)):($4+rn(0.01)) w p pt 0 ps 1 lt 2 lw 2 t 'U-Tree (value-based)'

## different styles

set samples 3
plot [0:1] \
     1*x w lp pt 1 lt 1t '1', \
     2*x w lp pt 2 lt 2t '2', \
     3*x w lp pt 3 lt 3t '3', \
     4*x w lp pt 4 lt 4t '4', \
     5*x w lp pt 5 lt 5t '5', \
     6*x w lp pt 6 lt 6t '6', \
     7*x w lp pt 7 lt 7t '7', \
     8*x w lp pt 8 lt 8t '8', \
     9*x w lp pt 9 lt 9t '9'

## output to file

set out 'plot.svg'; \
set term svg; \
replot; \
unset out; \
set term wxt

set out 'plot.pdf'; \
set term pdf; \
replot; \
unset out; \
set term wxt