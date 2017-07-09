rosrun tf static_transform_publisher 0 0 0 0 0 0 map camera_depth_optical_frame 100
rosrun dovlo_navigation wanderer
rviz -d viewDovloWanderer.rviz
rosbag play ../../dovlo_camera_depth_point-2017-06-25-11-48-31.bag
