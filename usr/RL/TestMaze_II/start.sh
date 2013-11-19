## define string for calling the program

## 2x2

# Learning curves
#start_string="-mode SPARSE -sample UNIFORM -nEp 2700 -maxTran 12 -minTrain 600 -maxTrain 1000 -maxTree 10000 -f 2 -l1 0.0002 -incr 200"
#start_string="-mode LINEAR_Q -sample UNIFORM -nEp 2700 -maxTran 12 -minTrain 600 -maxTrain 1000 -f 3 -l1 0.0005 -incr 200"
start_string="-mode LINEAR_Q_BELLMAN -sample UNIFORM -nEp 5000 -maxTran 12 -minTrain 200 -maxTrain 1000 -incr 200 -l1 0.001 -fincr 10 -dloss 1e-10 -alpha 5"
#start_string="-mode UTREE_PROB -sample UNIFORM -nEp 3000 -maxTran 12 -minTrain 600 -maxTrain 1000 -maxTree 10000 -incr 200"

# L1 sweep / UTree growth
#start_string="-mode SPARSE -sample UNIFORM -nEp 100 -maxTran 12 -minTrain 500 -maxTrain 500 -maxTree 10000 -f 2 -l1 0.005 -incr 0 -l1incr 0.005 -maxl1 0.1"
#start_string="-mode UTREE_PROB -sample UNIFORM -nEp 100 -maxTran 12 -minTrain 1500 -maxTrain 1500 -maxTree 10000 -incr 0 -utreegrowth t"

## 4x4

# Learning curves
#start_string="-mode SPARSE -sample UNIFORM -nEp 300 -maxTran 22 -minTrain 3000 -maxTrain 5000 -maxTree 50000 -l1 0.0005 -incr 1000 -fincr 50 -dl 0.0001"
#start_string="-mode UTREE_PROB -sample UNIFORM -nEp 400 -maxTran 22 -minTrain 500 -maxTrain 2000 -maxTree 50000 -incr 500"

# L1 sweep
#start_string="-mode SPARSE -sample UNIFORM -nEp 8 -maxTran 22 -minTrain 2000 -maxTrain 2000 -maxTree 50000 -l1 0.0001 -incr 0 -fincr 50 -dl 0.0001 -l1incr 0.0004 -maxl1 0.002"

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
