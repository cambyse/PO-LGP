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
