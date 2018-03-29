#include <iostream>
#include <fstream>
#include <chrono>
#include <algorithm>
#include <iterator>

#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include "pcl_ros/transforms.h"

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include "laser_geometry/laser_geometry.h"


using namespace std;
// using namespace std::chrono;

ros::Publisher pcl_from_scan;

// typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
laser_geometry::LaserProjection projector;

void hokuyo_callbacks(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointXYZI fake_hit;
    sensor_msgs::PointCloud2 cloud;
    projector.projectLaser(*scan_in, cloud);

    // convert message to pcl2
    pcl::fromROSMsg(cloud, *scan_cloud);

    for(int i=0;i<scan_cloud->size();i++) {
        pcl::PointXYZI p;
        p.x = scan_cloud->at(i).x;
        p.y = scan_cloud->at(i).y;
        p.z = scan_cloud->at(i).z;
        p.intensity = 1.0;
        cloud2->push_back(p);
    }

    // Add inf points
    for(int i=0;i<scan_in->ranges.size();i++) {
        if(isinf(scan_in->ranges[i])) {
            double r = 30.0;
            double theta = scan_in->angle_min + double(i) * scan_in->angle_increment;
            fake_hit.x = r * cos(theta);
            fake_hit.y = r * sin(theta);
            fake_hit.z = 0;
            fake_hit.intensity = -1.0;
            cloud2->push_back(fake_hit);
        }
    }

    // convert pcl2 back to message
    pcl::toROSMsg(*cloud2, cloud);

    // Publish the new point cloud.
    cloud.header.frame_id = "/laser";
    cloud.header.stamp = scan_in->header.stamp;
    pcl_from_scan.publish(cloud);

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "laserScan_to_pointcloud");
    ros::NodeHandle nh;

    ros::Subscriber hokuyo_sub;
    hokuyo_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, hokuyo_callbacks);

    
    pcl_from_scan = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("hokuyo_points", 1);

    while (ros::ok())
    {
        ros::spin();
    }

    nh.shutdown();          
    return 0;
}
