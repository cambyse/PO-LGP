echo "Setting ENV..."

export ROS_MASTER_URI=http://thecount.local:11311

UB=`lsb_release -rs`

if [ UB = "16.04" ]; then
	DEV=`udevadm info -e | grep -oE -m 1 "/en[[:alnum:]]*" | grep -oE "en[[:alnum:]]*"`
elif [ UB = "14.04" ]; then
	DEV="eth0"
fi
 
export ROS_IP=`ifconfig $DEV | grep -o "net [[:alpha:]]*:[[:graph:]]*" | grep -v '127.0.0.1' | cut -d: -f2 | awk '{ print $1}'`
# export ROS_HOSTNAME=`hostname`

echo ROS_HOSTNAME $ROS_HOSTNAME
echo ROS_MASTER_URI $ROS_MASTER_URI
echo "Using eth0"
echo ROS_IP $ROS_IP
echo "Done."
