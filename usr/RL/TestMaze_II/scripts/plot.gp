## activate sub- and superscripts, and macros
set termopt enhanced
set macros

## modulus
mod(x,y) = x-floor(x/y)*y

## random function
r(x) = 2*x*(rand(0)-0.5)
rn(x) = 0.6*x*invnorm(rand(0))

## plot styles
UNIQUE = "s u w lp pt 1 lt 1"
RAW = "w p pt 6 lt 2 ps 0.5"
SPLINE = "s ac w l lt 2"
FREQ = "s f w lp pt 2 lt 3"

## data folder
data_folder = '../evaluations/2013_10_ICML/'

########################################
## 4x4 Maze
########################################

## scale counts
f_div = 100

## data
data_file_1 = data_folder.'4x4_III_SPARSE_L1.txt'
BLOCK = "i 1"
C1 = "8"
C2 = "7"
rx = "rn(4e-5)"
ry = "rn(6e-3)"
#
data_file_1 = data_folder.'4x4_III_SPARSE.txt'
BLOCK = "i 3:10"
data_file_1 = data_folder.'4x4_III_UTREE_VALUE.txt'
BLOCK = ""
C1 = "2"
C2 = "7"
rx = "rn(4e1)"
ry = "rn(6e-3)"

## labels and tics
# set xlabel 'L^{1}-regularization'; \
unset y2tics; \
unset y2label; \
set xlabel 'Training Length'; \
set ylabel 'Mean Reward over 22 Steps'; \
set format y "%.2f"; \
set ytics 0,0.05; \
set mytics 5

plot [:][0:0.5] \
     data_file_1 u @C1:@C2 @BLOCK @UNIQUE t 'Mean reward', \
     data_file_1 u ($@C1+@rx):($@C2+@ry) @BLOCK @RAW t 'Mean reward raw data', \
     data_file_1 u @C1:(1./f_div) @BLOCK @FREQ t 'Number of data points / '.f_div
     # data_file_1 u @C1:@C2:(5e10) @BLOCK @SPLINE t ''

plot [:][0:0.5] \
     data_file_1 u @C1:@C2 @BLOCK @UNIQUE t 'Mean reward'

plot [:3000][0:] \
     data_folder.'4x4_III_SPARSE.txt' u @C1:@C2 i 3:10 s u w lp pt 1 lt 1 t 'Mean reward', \
     data_folder.'4x4_III_SPARSE.txt' u ($@C1+@rx):($@C2+@ry) i 3:10 w p pt 6 lt 2 ps 0.5 t 'Mean reward raw data', \
     data_folder.'4x4_III_SPARSE.txt' u @C1:(1./f_div) i 3:10 s f w lp pt 2 lt 3 t 'No data points / '.f_div, \
     data_folder.'4x4_III_UTREE_VALUE.txt' u @C1:@C2 s u w lp pt 1 lt 4 t 'Mean reward', \
     data_folder.'4x4_III_UTREE_VALUE.txt' u ($@C1+@rx):($@C2+@ry) w p pt 6 lt 5 ps 0.5 t 'Mean reward raw data', \
     data_folder.'4x4_III_UTREE_VALUE.txt' u @C1:(1./f_div) s f w lp pt 2 lt 6 t 'No data points / '.f_div

plot [0:5100][0:] \
     data_folder.'4x4_III_SPARSE.txt' u 2:4 s u w lp pt 1 lt 1 t 'CRF', \
     data_folder.'4x4_III_UTREE_VALUE.txt' u 2:4 s u w lp pt 1 lt 4 t 'UTree (value)'

plot [0:5100][0:30] \
     data_folder.'4x4_III_SPARSE.txt' u 2:(1) s f w lp pt 1 lt 1 t 'CRF', \
     data_folder.'4x4_III_UTREE_VALUE.txt' u 2:(1) s f w lp pt 1 lt 4 t 'UTree (value)'

########################################
## 2x2 Maze
########################################

## Learning Curves
set xlabel "Number of Random Transitions"; \
set ylabel "Mean Reward"; \
set ytics; \
unset y2label; \
unset y2tics; \
set ytics auto
plot [0:1000] \
     data_folder.'LINEAR_Q_tmp.txt'	u 2:4 s u w lp t 'Linear-Q', \
     data_folder.'SPARSE_tmp.txt' 	u 2:4 s u w lp t 'CRF', \
     data_folder.'2x2_UTREE_PROB.txt' 	u 2:4 s u w lp t 'UTree (prob.)', \
     data_folder.'2x2_UTREE_VALUE.txt' 	u 2:4 s u w lp t 'UTree (value)'
plot [0:1000] data_folder.'2x2_UTREE_PROB.txt' 	u ($2+rn(20)):($4+rn(0.01))  t 'UTree (prob.)'
plot [0:1000] data_folder.'2x2_UTREE_VALUE.txt' u ($2+rn(20)):($4+rn(0.01))  t 'UTree (value)'
plot [0:1000] \
     data_folder.'2x2_LINEAR_Q.txt'	u 2:(1) s f w lp t 'Linear-Q', \
     data_folder.'2x2_SPARSE.txt' 	u 2:(1) s f w lp t 'CRF', \
     data_folder.'2x2_UTREE_PROB.txt' 	u 2:(1) s f w lp t 'UTree (prob.)', \
     data_folder.'2x2_UTREE_VALUE.txt' 	u 2:(1) s f w lp t 'UTree (value)'

## L1-regularization
## -maxTrain 500 in block 0; -maxTrain 1500 in block 1
set xlabel "Number of Features"
set ylabel "Mean Reward"
set y2label "L^1-regularization"
set y2tics
set ytics nomirror
plot \
     data_folder.'2x2_SPARSE_L1_F3.txt'	       u 12:4 s u w lp lt 1 pt 2 t 'Mean Reward', \
     data_folder.'2x2_SPARSE_L1_F3.txt'	       u 12:10 s u w lp lt 2 pt 2 t 'L^1-regularization' axes x1y2
plot \
     data_folder.'2x2_SPARSE_L1_F2.txt'	       u 12:4 s u w lp lt 1 pt 2 t 'Mean Reward', \
     data_folder.'2x2_SPARSE_L1_F2.txt'	       u 12:10 s u w lp lt 2 pt 2 t 'L^1-regularization' axes x1y2

## UTree size
set xlabel "Number of Leaves"
set ylabel "Mean Reward"
set y2label "chi^2-score"
set y2tics
set ytics nomirror
plot \
     data_folder.'2x2_UTREE_PROB_growth.txt'    u (($6-1)/2):4 s u w lp t 'Mean Reward', \
     data_folder.'2x2_UTREE_PROB_growth.txt'    u (($6-1)/2):8 s u w lp t 'chi^2-score' axes x1y2

set y2label "K-S-score"
plot \
     data_folder.'2x2_UTREE_VALUE_growth.txt'    u (($6-1)/2):4 s u w lp t 'Mean Reward', \
     data_folder.'2x2_UTREE_VALUE_growth.txt'    u (($6-1)/2):8 s u w lp t 'K-S-score' axes x1y2

set out 'plot.pdf'; \
set term pdf; \
replot; \
unset out; \
set term wxt