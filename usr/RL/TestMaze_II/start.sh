start_string="-m VALUE_BASED_UTREE --minT 100 --maxT 200 --incT 10 -e 100 -r 10"

## print kind of help if no arguments are given
if [ $# -lt 1 ]; then
    echo "expecting at least one argument"
    echo "    { verbose | quiet } [ OMP_NUM_THREADS ]"
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
    if [ -e ./BatchWorker_verbose/BatchWorker_verbose ]; then
	eval "./BatchWorker_verbose/BatchWorker_verbose $start_string"
    else
	echo "Error: BatchWorker_verbose does not exist"
    fi
else
    if [ -e ./BatchWorker_quiet/BatchWorker_quiet ]; then
	eval "nohup ./BatchWorker_quiet/BatchWorker_quiet $start_string &"
#	eval "./BatchWorker_quiet/BatchWorker_quiet $start_string"
    else
	echo "Error: BatchWorker_quiet does not exist"
    fi
fi
