export ROS_MASTER_URI=http://192.168.0.23:11311/
export ROS_IP=192.168.0.23
source `dirname ${BASH_SOURCE[0]}`/../common/catkin_ws/devel/setup.bash
roscore &

