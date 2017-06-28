export ROS_MASTER_URI=http://localhost:11311/
unset ROS_IP
source `dirname ${BASH_SOURCE[0]}`/../common/catkin_ws/devel/setup.bash
roscore &

