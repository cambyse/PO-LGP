## define string for calling the program

## 2x2
# Learning curves
#start_string="-mode LINEAR_Q -sample UNIFORM -nEp 2000 -maxTran 12 -minTrain 10 -maxTrain 200 -f 3 -l1 0.0005 -incr 10"
#start_string="-mode UTREE_VALUE -sample UNIFORM -nEp 200 -maxTran 12 -minTrain 200 -maxTrain 200 -maxTree 10000 -incr 0"
#start_string="-mode SPARSE -sample UNIFORM -nEp 1000 -maxTran 12 -minTrain 110 -maxTrain 190 -maxTree 10000 -f 2 -l1 0.0002 -incr 10"
# L1 sweep / UTree growth
#start_string="-mode SPARSE -sample UNIFORM -nEp 100 -maxTran 12 -minTrain 1500 -maxTrain 1500 -maxTree 10000 -f 3 -l1 0.005 -incr 0 -l1incr 0.005 -maxl1 0.1"
start_string="-mode UTREE_PROB -sample UNIFORM -nEp 100 -maxTran 12 -minTrain 1500 -maxTrain 1500 -maxTree 10000 -incr 0 -utreegrowth t"

## 4x4
#start_string="-mode SPARSE -sample UNIFORM -nEp 1000 -maxTran 22 -minTrain 100 -maxTrain 1000 -maxTree 50000 -l1 0.001 -incr 100 -fincr 50 -dl 0.0001"
#start_string="-mode UTREE_PROB -sample UNIFORM -nEp 2000 -maxTran 22 -minTrain 500 -maxTrain 10000 -maxTree 50000 -incr 500"

## print kind of help if no arguments are given
if [ $# -lt 1 ]; then
    echo "expecting at least one argument"
    echo "    [ verbose | quiet [ OMP_NUM_THREADS ] ]"
    exit 0
fi

## print error message if first argument doesn't match
if [ "$1" != "verbose" -a "$1" != "quiet" ]; then
    echo "first argument must be 'verbose' or 'quiet'"
    exit 0
fi

## print error message if second argument doesn't match
## set number of threads otherwise
if [ $# -gt 1 ]; then
    if [ $2 -eq $2 2> /dev/null ]; then
	export OMP_NUM_THREADS=$2
    else
	echo "second argument must be a number"
	exit 0
    fi
else
    unset OMP_NUM_THREADS
fi

## call the program
if [ "$1" == "verbose" ]; then
    if [ -e TestMaze_II_verbose ]; then
	eval "./TestMaze_II_verbose $start_string"
    else
	eval "./TestMaze_II $start_string"
    fi
else
    if [ -e TestMaze_II_quiet ]; then
	eval "nohup ./TestMaze_II_quiet $start_string &"
    else
	eval "nohup ./TestMaze_II $start_string &"
    fi
fi