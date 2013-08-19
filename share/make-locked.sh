#/bin/sh

if mkdir $1/Make.lock 2> /dev/null
then    # directory did not exist, but was created successfully
    echo "                                         ***** Make " $1
    make --silent -C $1
    rm -rf $1/Make.lock
else
    echo "                                         ***** Waiting " $1
    #printf "for other make to complete"
    while [ -d $1/Make.lock ]
    do
	sleep 0.2
	#printf "." 
    done
    #printf "done\n"
fi
