#include <ros/ros.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
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

ros::Publisher pub_debug;
ros::Publisher pub_marker;
ros::Publisher pub_depth_goal;
pcl::ConditionalRemoval<pcl::PointXYZ> space_filter;

visualization_msgs::Marker marker;

#define Z_BACK   0.5
#define Z_STOP   0.5
#define Z_FULL   0.7
#define X_LEFT  -0.1
#define X_RIGHT -0.1
#define MAX_X    0.1
#define MAX_Z    0.3

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
    voxel_filter.setLeafSize(0.04,0.04,0.04);
    voxel_filter.filter(*cloudXYZ_1);

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_filter;
    radius_filter.setInputCloud(cloudXYZ_1);
    radius_filter.setRadiusSearch(0.08);
    radius_filter.setMinNeighborsInRadius (9);
    radius_filter.filter(*cloudXYZ_2);

    // Find closest obstacle in left, in front, and right
    float minZ_left  = 10;
    float minZ_mid   = 10;
    float minZ_right = 10;
    for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloudXYZ_2->points.begin(); it < cloudXYZ_2->points.end(); it++) {
        if ( it->x < -0.1 ) {
            if ( it->z < minZ_left ) minZ_left = it->z;
        } else if (it->x > 0.1) {
            if ( it->z < minZ_right ) minZ_right = it->z;
        } else {
            if ( it->z < minZ_mid ) minZ_mid = it->z;
        }
    }
    //std::cout << "L:" << minZ_left << " M:" << minZ_mid << " L:" << minZ_right << std::endl;

    // Choose goal based on obstacle position

    marker.points[0].z = minZ_left;
    marker.points[1].z = minZ_mid;
    marker.points[2].z = minZ_right;
    marker.header.frame_id = cloud_msg->header.frame_id;
    marker.header.stamp = ros::Time::now();
    pub_marker.publish(marker);

    // Back if mid<0.3
    // FullZ if mid>1
    // Back if mid <0.1
    // Percentage with between 1:0.3
    float goalZ;
    float goalX;
    if (minZ_mid<Z_BACK) {
        goalX = 0;
        goalZ = -MAX_Z;
    } else {
        float sp = minZ_mid>Z_FULL ? 1 : (minZ_mid<Z_STOP ? 0 : (minZ_mid-Z_STOP)/(Z_FULL-Z_STOP));
        goalZ = MAX_Z*sp;
        goalX = MAX_X*sqrt(1-sp)*(minZ_right<minZ_left?-1:1);
    }

    geometry_msgs::PointStamped depth_goal;
    depth_goal.header.frame_id = cloud_msg->header.frame_id;
    //depth_goal.header.stamp = ros::Time::now();
    depth_goal.point.x = -goalX;
    depth_goal.point.y = 0;
    depth_goal.point.z = goalZ;
    pub_depth_goal.publish(depth_goal);
        
    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toPCLPointCloud2(*cloudXYZ_2, *cloudPCL);
    pcl_conversions::moveFromPCL(*cloudPCL,output);
    pub_debug.publish(output);

}

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc,argv,"dovlo_wanderer");
    ros::NodeHandle nh;

    // Create a ROS subscribers
    ros::Subscriber sub = nh.subscribe("camera/depth/points",1,cloud_cb);

    // Create a ROS publishers
    pub_debug       = nh.advertise<sensor_msgs::PointCloud2>("debug_points",1);
    pub_marker      = nh.advertise<visualization_msgs::Marker>("debug_marker",1);
    pub_depth_goal  = nh.advertise<geometry_msgs::PointStamped>("depth_follower/goal",1);
    
    // Create the space restriction
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());
    range_cond->addComparison ( pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (
        new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.1)
    ));
    range_cond->addComparison ( pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (
        new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 2)
    ));
    range_cond->addComparison ( pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (
        new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, -0.5)
    ));
    range_cond->addComparison ( pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (
        new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, 0.05)
    ));
    range_cond->addComparison ( pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (
        new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, -0.5)
    ));
    range_cond->addComparison ( pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (
        new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, 0.5)
    ));
    space_filter.setCondition (range_cond);

    // Marker initialization

    marker.ns = "z_points";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    for (int i=0;i<3;i++) {
        geometry_msgs::Point p;
        p.x = (i-1)*0.2;
        p.y = 0;
        p.z = 0;
        marker.points.push_back(p);
    }

    // Spin
    ros::spin();
}
