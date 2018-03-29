#include <iostream>
#include <fstream>
#include <chrono>
#include <algorithm>
#include <iterator>

#include <ros/ros.h>
#include <ros/time.h>
#include "std_msgs/Time.h"
#include "dynamixel_msgs/JointState.h"
#include "sensor_msgs/PointCloud2.h"

// Services
#include "laser_assembler/AssembleScans2.h"


using namespace std;
using namespace laser_assembler;
// using namespace std::chrono;

ros::Time start_record_time;
ros::Time end_record_time;
bool go = false;

//call back for start time, saves in global variable
void startTime(const std_msgs::Time &msg) {
    start_record_time = msg.data;
    std::cout << "startTime recorded" << std::endl;
}


//callback for end time, saves in global variable and updates go to start compilation
void endTime(const std_msgs::Time &msg) {
    end_record_time = msg.data;
    go = true;
    std::cout << "endtime recorded" << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Subscriber_to_time");
    ros::NodeHandle nh;

    ros::Subscriber sub_1 = nh.subscribe("/time/start_time", 1, &startTime);
    ros::Subscriber sub_2 = nh.subscribe("/time/end_time", 1, &endTime);
    ros::Publisher  pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("assembled_cloud", 1);
    ros::ServiceClient client_ =  nh.serviceClient<AssembleScans2>("assemble_scans2");

    AssembleScans2 srv;

    // nh.param<std::string>("assembled_cloud_mode", assembled_cloud_mode, "subscriber");
    // nh.param<double>("scan_time", scan_time, 14);
    nh.setParam("/fixed_frame", "map");
    nh.setParam("/max_scans", 600);
    nh.setParam("/tf_cache_time_secs", 20);

    ros::topic::waitForMessage<dynamixel_msgs::JointState>("/tilt_controller/state", ros::Duration(20)); 
    ros::service::waitForService("assemble_scans");
    
    while (ros::ok())
    {
        ros::spinOnce();
        ros::Duration(0.25).sleep();
        if(go) {
            go = false;
            srv.request.begin = start_record_time;
            srv.request.end   = end_record_time;

            // Make the service call
            if (client_.call(srv)) {
                ROS_INFO_STREAM("Published Cloud using service") ;
                pub_cloud.publish(srv.response.cloud);
            }
            else {
                ROS_ERROR("Error making service call\n") ;
            } 
        }
        // double total_sec = end_record_time.toSec() - start_record_time.toSec();
        // std::cout << '\r' << "total time : " << total_sec << std::flush;
    }

    nh.shutdown();          
    return 0;
}


        
        

