echo "Setting ENV..."

export ROS_MASTER_URI=http://bigbirdc1.informatik.uni-stuttgart.de:11311
export ROS_IP=`ifconfig eth0 | grep 'inet addr:'| grep -v '127.0.0.1' | cut -d: -f2 | awk '{ print $1}'`
# export ROS_HOSTNAME=`hostname`
export ROS_HOSTNAME=$ROS_IP

echo ROS_HOSTNAME $ROS_HOSTNAME
echo ROS_MASTER_URI $ROS_MASTER_URI
echo "Using eth0"
echo ROS_IP $ROS_IP

echo "...done"
