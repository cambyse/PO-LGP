epsilon=.01

while [ "$epsilon" != "1.00" ]; do
    ./TestMaze_II optimal $epsilon 0.9 &
    epsilon=`echo $epsilon+.01 | bc`
    sleep 1
done