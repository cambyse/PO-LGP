## define string for calling the program
start_string="./TestMaze_II -mode SPARSE -sample UNIFORM -nEp 100 -maxTran 22 -minTrain 500 -maxTrain 500 -maxTree 50000 -l1 0.0005 -incr 0 -fincr 50 -dl 0.0001"

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
    eval "$start_string"
else
    eval "nohup $start_string &"
fi