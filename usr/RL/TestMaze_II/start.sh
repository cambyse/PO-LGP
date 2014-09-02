
## Minimal Maze
# start_string="--env Minimal -m TEL --minT 200 --maxT 200 --incT 0 --minH -2 --maxH -1 --extH 2 --minCycles -3 --maxCycles 3 -e 32 -r 50 --l1 1e-4 --delta 1e-10"
# start_string="--env Minimal -m VALUE_BASED_UTREE --minT 600 --maxT 1000 --incT 200 -e 32 -r 500"
# start_string="--env Minimal -m MODEL_BASED_UTREE --minT 400 --maxT 400 --incT 0 -e 32 -r 25"
# start_string="--env Minimal -m TEM --minT 600 --maxT 1000 --incT 200 --minH -2 --maxH 0 --extH 2 --minCycles -2 --maxCycles 2 -e 32 -r 50 --l1 1e-5 --delta 1e-5"

## 4x4_III Maze
start_string="--env 4x4_III -m TEL --minT 3000 --maxT 5000 --incT 1000 --minH -3 --maxH -1 --extH 1 --minCycles 0 --maxCycles 4 -e 22 -r 10 --l1 1e-4 --delta 1e-10"
start_string="--env 4x4_III -m TEM --minT 3000 --maxT 5000 --incT 1000 --minH -3 --maxH  0 --extH 1 --minCycles 0 --maxCycles 3 -e 22 -r 50 --l1 1e-5 --delta 1e-5"
start_string="--env 4x4_III -m VALUE_BASED_UTREE --minT 2000 --maxT 5000 --incT 1000 -e 22 -r 300"
start_string="--env 4x4_III -m MODEL_BASED_UTREE --minT 3000 --maxT 5000 --incT 1000 -e 22 -r 300 -t 20000"


## Button World
# start_string="--env sep-button --button_n 3 -m               TEM --minT  2 --maxT  10 --incT  1 --minH -1 --maxH 0 --extH 1 --minCycles 2 -e 10 -r  25 --l1 0.0001 --delta 1e-5 -t 20000"
# start_string="--env sep-button --button_n 3 -m               TEL --minT  10 --maxT  10 --incT  1 --minH -1 --maxH 0 --extH 1 --minCycles 3 -e 10 -r  100 --l1  1e-9 --delta 1e-10"
# start_string="--env sep-button --button_n 3 -m VALUE_BASED_UTREE --minT  20 --maxT  100 --incT  10                                           -e 10 -r 500                          "
# start_string="--env joint-button --button_n 3 -m            RANDOM --minT 1 --maxT 1  --incT 0                                           -e 10 -r 500                        "

## Cheese Maze
start_string="--env cheese -m TEM --minT 200 --maxT 200 --incT 0 --minH -2 --maxH  0 --extH 1 --maxCycles 3 -e 100 -r 100 --l1 1e-4 --delta 1e-5"

## print kind of help if no arguments are given
if [ $# -lt 1 ]; then
    echo "expecting at least one argument"
    echo "    { verbose | quiet } [ OMP_NUM_THREADS ] [ OMP_NESTED=FALSE ]"
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

## print error message if third argument is not TRUE or FALSE
## enable/disable nested parallelism otherwise
if [ $# -gt 2 ]; then
    if [ "$3" = "TRUE" -o "$3" = "FALSE" ]; then
	export OMP_NESTED="$3"
    else
	echo "third argument must be 'TRUE' or 'FALSE'"
	exit 0
    fi
else
    export OMP_NESTED="FALSE"
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
