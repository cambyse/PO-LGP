set title 'Creating a Series of Instances'
set xlabel 'elapsed time/ms'
set ylabel 'number of items created'
plot \
     'Instance_Performance.txt' i 0 s u w lp pt 6 t 'w/o assignmnet, w/o container', \
     'Instance_Performance.txt' i 1 s u w lp pt 6 t 'w assignmnet, w/o container', \
     'Instance_Performance.txt' i 2 s u w lp pt 6 t 'w/o assignmnet, w    container', \
     'Instance_Performance.txt' i 3 s u w lp pt 6 t 'w assignmnet, w    container'

set term jpeg
set out 'Instance_Initialization.jpg'
replot
set term wxt

#==================================================#

set title 'Setting/Unsetting a Container for a Series of Instances'
set xlabel 'number of items'
set ylabel 'elapsed time/ms'
plot 'Instance_Performance.txt' i 4 u 1:2 w lp pt 6 t 'set container', 'Instance_Performance.txt' i 4 u 1:3 w lp pt 6 t 'unset container'

set term jpeg
set out 'Instance_Set-Unset_Container.jpg'
replot
set term wxt

#==================================================#

set title 'Iterating Through a Series of Instances'
set xlabel 'number of items iterated'
set ylabel 'elapsed time/ms'
plot 'Instance_Performance.txt' i 4 u 1:4 w lp pt 6 t 'iterate with container', 'Instance_Performance.txt' i 4 u 1:6 w lp pt 6 t 'iterate without container'

set term jpeg
set out 'Instance_Iteration.jpg'
replot
set term wxt

#==================================================#

set title 'Random Access within a Series of Instances'
set xlabel 'number of items'
set ylabel 'time for random access time/ms'
plot [0:] 'Instance_Performance.txt' i 4 u 1:5 w lp pt 6 t 'with container', 'Instance_Performance.txt' i 4 u 1:7 w lp pt 6 t 'without container'

set term jpeg
set out 'Instance_Random_Access.jpg'
replot
set term wxt