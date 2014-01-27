#/bin/sh

cd $1
LASTFILE=`ls -Art | tail -n 1`
if [ $LASTFILE = '.lastMake' ]; then
    echo "                                         ***** UpToDate " $1
else
    if mkdir Make.lock 2> /dev/null
    then    # lock did not exist and was created successfully
	echo "                                         ***** Make " $1
	make
	if [ $? -eq 0 ] ; then
            date +'%y-%m-%d-%T' > .lastMake
	else
	    echo "                                         ***** FAILED " $1
	fi
	rm -rf Make.lock
    else
	echo "                                         ***** Waiting " $1
	while [ -d $1/Make.lock ]
	do
	    sleep 0.2
	done
    fi
fi
