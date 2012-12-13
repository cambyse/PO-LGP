epsilon=.01
discount=.01

while [ "$discount" != "1.00" ]; do

    ./TestMaze_II optimal $epsilon $discount &

    epsilon=`echo $epsilon+.01 | bc`
    if [ "$epsilon" == "1.00" ]; then
	epsilon=.01
	discount=`echo $discount+.01 | bc`
    fi

done