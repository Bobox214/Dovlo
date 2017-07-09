//
// PCL from a camera/depth/points as an input
// Do spatial filtering
// Do 1cm voxel filtering
// Publish output

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
#include <math.h>

ros::Publisher pub_output;
pcl::ConditionalRemoval<pcl::PointXYZ> space_filter;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // Containter for original and filtered data
    pcl::PCLPointCloud2* cloudPCL = new pcl::PCLPointCloud2; 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ_1 (new pcl::PointCloud<pcl::PointXYZ>()); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ_2 (new pcl::PointCloud<pcl::PointXYZ>()); 

    // Convert input to PCL
    pcl_conversions::toPCL(*cloud_msg,*cloudPCL);
    pcl::fromPCLPointCloud2(*cloudPCL, *cloudXYZ_1);

    // Perform filtering
    space_filter.setInputCloud (cloudXYZ_1);
    space_filter.setKeepOrganized(true);
    space_filter.filter (*cloudXYZ_2);

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloudXYZ_2);
    voxel_filter.setLeafSize(0.01,0.01,0.01);
    voxel_filter.filter(*cloudXYZ_1);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toPCLPointCloud2(*cloudXYZ_1, *cloudPCL);
    pcl_conversions::moveFromPCL(*cloudPCL,output);
    pub_output.publish(output);

}

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc,argv,"pclSimplify");
    ros::NodeHandle nh;

    // Create a ROS subscribers
    ros::Subscriber sub = nh.subscribe("camera/depth/points",1,cloud_cb);

    // Create a ROS publishers
    pub_output      = nh.advertise<sensor_msgs::PointCloud2>("debug_points",1);
    
    // Create the space restriction
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());
    range_cond->addComparison ( pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (
        new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0)
    ));
    range_cond->addComparison ( pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (
        new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 2)
    ));
    range_cond->addComparison ( pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (
        new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, -0.5)
    ));
    range_cond->addComparison ( pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (
        new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, 0.2)
    ));
    range_cond->addComparison ( pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (
        new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, -0.5)
    ));
    range_cond->addComparison ( pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (
        new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, 0.5)
    ));
    space_filter.setCondition (range_cond);

    // Spin
    ros::spin();
}
