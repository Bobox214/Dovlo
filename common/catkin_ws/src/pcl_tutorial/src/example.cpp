#include <ros/ros.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <iostream>

ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // Containter for original and filtered data
    pcl::PCLPointCloud2*        cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2*        cloud1 = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloud1Ptr(cloud1);
    pcl::PCLPointCloud2*        cloud2 = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloud2Ptr(cloud2);
    pcl::PCLPointCloud2         cloud3;

    // Convert input to PCL
    pcl_conversions::toPCL(*cloud_msg,*cloud);

    // plane filtering
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    pass.setInputCloud(cloudPtr);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-1, 0.05);
    pass.filter(*cloud1);
    //pass.setInputCloud(cloud1Ptr);
    //pass.setFilterFieldName("x");
    //pass.setFilterLimits(-1, 1);
    //pass.filter(*cloud2);
    //pass.setInputCloud(cloud2Ptr);
    //pass.setFilterFieldName("z");
    //pass.setFilterLimits(0.2, 2);
    //pass.filter(*cloud1);

    //// Perform filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud1Ptr);
    sor.setLeafSize(0.04,0.04,0.04);
    sor.filter(*cloud2);

    pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> stat;
    stat.setInputCloud(cloud2Ptr);
    stat.setMeanK(50);
    stat.setStddevMulThresh(0.1);
    stat.filter(*cloud1);

    pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> outrem;
    outrem.setInputCloud(cloud1Ptr);
    outrem.setRadiusSearch(0.04);
    outrem.setMinNeighborsInRadius (3);
    outrem.filter (cloud3);



    //std::cout << cloud3.width << "*" << cloud3.height << " nData:"<< cloud3.data.size() << std::endl;
    //std::cout << cloud3.fields[0] << " type " << int(cloud3.fields[0].datatype) << std::endl;
    //std::cout << cloud3.fields[1] << " type " << int(cloud3.fields[1].datatype) << std::endl;
    //std::cout << cloud3.fields[2] << " type " << int(cloud3.fields[2].datatype) << std::endl;

    float minZ = 10;
    for(int i=0;i<cloud3.data.size();i+=cloud3.point_step) {
        float value;
        memcpy (&value, reinterpret_cast<const char*> (&cloud3.data[i+8]), sizeof (float));
        if (value<minZ) minZ = value;
    }
    std::cout << minZ << std::endl;
    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::moveFromPCL(cloud3,output);

    //ROS_INFO("PointCloud In  : %d*%d ", cloud->width , cloud->height );
    //ROS_INFO("PointCloud Out : %d*%d nData:%d", cloud3.width , cloud3.height ,cloud3.data.size());

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
