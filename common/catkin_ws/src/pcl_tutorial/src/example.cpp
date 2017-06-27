#include <ros/ros.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // Containter for original and filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    // Convert input to PCL
    pcl_conversions::toPCL(*cloud_msg,*cloud);

    // Perform filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloudPtr);
    sor.setLeafSize(0.05,0.05,0.05);
    sor.filter(cloud_filtered);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::moveFromPCL(cloud_filtered,output);

    ROS_INFO("PointCloud : %d*%d ", cloud->width , cloud->height );
    ROS_INFO("PointCloud after filtering: %d*%d ", cloud_filtered.width , cloud_filtered.height );

    pub.publish(output);
}

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc,argv,"pcl_tutorial");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("input",1,cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("output",1);

    // Spin
    ros::spin();
}
