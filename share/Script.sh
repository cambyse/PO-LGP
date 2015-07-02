echo "Setting ENV..."

export ROS_MASTER_URI=http://bigbirdc1.informatik.uni-stuttgart.de:11311
export ROS_IP=`ifconfig mlan0 | grep 'inet addr:'| grep -v '127.0.0.1' | cut -d: -f2 | awk '{ print $1}'`

echo ROS_MASTER_URI $ROS_MASTER_URI
echo "Using eth0"
echo ROS_IP $ROS_IP

echo "...done"
