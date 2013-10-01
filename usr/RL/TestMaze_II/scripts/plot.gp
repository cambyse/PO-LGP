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

########################################
## 4x4 Maze
########################################

## scale counts
f_div = 100

## data
data_file_1 = '../4x4_III_SPARSE_L1.txt'
BLOCK = "i 1"
C1 = "8"
C2 = "7"
rx = "rn(4e-5)"
ry = "rn(6e-3)"
#
data_file_1 = '../4x4_III_SPARSE.txt'
BLOCK = "i 3:10"
data_file_1 = '../4x4_III_UTREE_VALUE.txt'
BLOCK = ""
C1 = "2"
C2 = "7"
rx = "rn(4e1)"
ry = "rn(6e-3)"

## labels and tics
# set xlabel 'L^{1}-regularization'; \
set xlabel 'Training Length'; \
set ylabel 'Mean Reward over 22 Steps'; \
set format y "%.2f"; \
set ytics 0,0.05; \
set mytics 5

plot [:][0:0.5] \
     data_file_1 u @C1:@C2 @BLOCK @UNIQUE t 'Mean reward', \
     data_file_1 u ($@C1+@rx):($@C2+@ry) @BLOCK @RAW t 'Mean reward raw data', \
     data_file_1 u @C1:(1./f_div) @BLOCK @FREQ t 'No data points / '.f_div
     # data_file_1 u @C1:@C2:(5e10) @BLOCK @SPLINE t ''


plot [:3000][0:] \
     '../4x4_III_SPARSE.txt' u @C1:@C2 i 3:10 s u w lp pt 1 lt 1 t 'Mean reward', \
     '../4x4_III_SPARSE.txt' u ($@C1+@rx):($@C2+@ry) i 3:10 w p pt 6 lt 2 ps 0.5 t 'Mean reward raw data', \
     '../4x4_III_SPARSE.txt' u @C1:(1./f_div) i 3:10 s f w lp pt 2 lt 3 t 'No data points / '.f_div, \
     '../4x4_III_UTREE_VALUE.txt' u @C1:@C2 s u w lp pt 1 lt 4 t 'Mean reward', \
     '../4x4_III_UTREE_VALUE.txt' u ($@C1+@rx):($@C2+@ry) w p pt 6 lt 5 ps 0.5 t 'Mean reward raw data', \
     '../4x4_III_UTREE_VALUE.txt' u @C1:(1./f_div) s f w lp pt 2 lt 6 t 'No data points / '.f_div

########################################
## 2x2 Maze
########################################

## Learning Curves
set xlabel "Number of Random Transitions"
set ylabel "Mean Reward"
unset y2label
unset y2tics
set ytics mirror
plot [0:1000] \
     '../2x2_LINEAR_Q.txt'             u 2:4 s u w lp t 'Linear-Q', \
     '../2x2_SPARSE.txt'	       u 2:4 s u w lp t 'CRF', \
     '../2x2_UTREE_PROB.txt'	       u 2:4 s u w lp t 'UTree (prob.)', \
     '../2x2_UTREE_VALUE.txt'	       u 2:4 s u w lp t 'UTree (value)'

## L1-regularization
## -f 2, -maxTrain 500 in block 0; -f 3, -maxTrain 1500 in block 1
set xlabel "Number of Features"
set ylabel "Mean Reward"
set y2label "L^1-regularization"
set y2tics
set ytics nomirror
plot \
     '../2x2_SPARSE_L1.txt'	       u 12:4 i 0 s u w lp lt 1 pt 2 t 'Mean Reward', \
     '../2x2_SPARSE_L1.txt'	       u 12:10 i 0 s u w lp lt 2 pt 2 t 'L^1-regularization' axes x1y2

## UTree size
set xlabel "Number of Leaves"
set ylabel "Mean Reward"
set y2label "chi^2-score"
set y2tics
set ytics nomirror
plot \
     '../2x2_UTREE_PROB_growth.txt'    u (($6-1)/2):4 s u w lp t 'Mean Reward', \
     '../2x2_UTREE_PROB_growth.txt'    u (($6-1)/2):8 s u w lp t 'chi^2-score' axes x1y2

set y2label "K-S-score"
plot \
     '../2x2_UTREE_VALUE_growth.txt'    u (($6-1)/2):4 s u w lp t 'Mean Reward', \
     '../2x2_UTREE_VALUE_growth.txt'    u (($6-1)/2):8 s u w lp t 'K-S-score' axes x1y2