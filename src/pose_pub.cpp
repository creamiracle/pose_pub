#include <fstream>
#include <sstream>
#include <iostream>
#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>

visualization_msgs::MarkerArray poseMarker;

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "pose_pub");

    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    ros::Publisher pose_pub_marker = n.advertise<visualization_msgs::MarkerArray>("/robot_position_marker", 1);

    std::ifstream inf;
    inf.open("/home/lin/catkin_ws/src/detect_human/result/resultfeb.txt", std::ifstream::in);
 
    const int cnt = 3;          
 
    std::string line;
     
    //int i = 0;
    int j = 0;
    size_t comma = 0;
    size_t comma1 = 0;
    size_t comma2 = 0; 
    int count = 0;

    while (!inf.eof())
    {
        //show in a MarkerArray
        visualization_msgs::Marker aMarker;
        aMarker.header.frame_id = "/map";
        aMarker.header.stamp = ros::Time::now();  
        aMarker.type = aMarker.CUBE;
        aMarker.scale.x = 0.05;
        aMarker.scale.y = 0.05;
        aMarker.scale.z = 0.05;
        aMarker.color.a = 1;
        aMarker.color.g = 1;
        aMarker.id = count;
        aMarker.ns = "sector";

        getline(inf,line);
        
        comma = line.find(',',0);
        double comma_ = std::atof(line.substr(0,comma).c_str());
        aMarker.pose.position.x = comma_;

        while (comma < line.size() && j == 0)
        {
            comma1 = line.find(',',comma + 1);
            double comma1_ = std::atof(line.substr(comma + 1,comma1-comma-1).c_str());
            aMarker.pose.position.y = comma1_;
            ++j;
            comma = comma1;
        }
        while (comma < line.size() && j == 1)
        {
            comma2 = line.find(',',comma + 1);
            double comma2_ = std::atof(line.substr(comma + 1,comma2-comma-1).c_str());
            aMarker.pose.position.z = 0;
            ++j;
        }
        
        j = 0;
        count++;
        poseMarker.markers.push_back(aMarker);
        ROS_INFO("position %d is %f, %f, %f",count, aMarker.pose.position.x, aMarker.pose.position.y, aMarker.pose.position.z);
    }
 
    inf.close();

    while(ros::ok())
    {
        pose_pub_marker.publish(poseMarker);
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
     
    return 0;
} 