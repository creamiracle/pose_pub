#include <fstream>
#include <sstream>
#include <iostream>
#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cmath>

typedef struct human_position
{
    double x;
    double y;
    double sec;
    double nsec;
    double gaussian;
}human;

typedef struct map_grid
{
    int coords[2];
    double human_dendity;
}maps;

visualization_msgs::MarkerArray poseMarker;
visualization_msgs::MarkerArray newPoseMarker;
visualization_msgs::MarkerArray robotPoseMarker;

nav_msgs::OccupancyGrid map;

std::vector<human> human_list;
std::vector<human> new_human_list;
std::vector<geometry_msgs::Pose> robotPose;

int count = 0;
int count_ = 0;
int count1 = 0;
int map_width = 0;
int map_height = 0;
float map_resolution = 0.0;

//calculate gaussian value
double gaussian2D(double x, double xc, double y, double yc, double sigma)
{
    double exponent = (std::pow(x - xc, 2) + std::pow(y - yc, 2)) / (2 * std::pow(sigma, 2));
    return std::exp(-exponent);
}

//make marker array 
void makeMarkerArray(double x, double y, int count, double g, double b, visualization_msgs::MarkerArray markerArray)
{
    visualization_msgs::Marker aMarker;
    aMarker.header.frame_id = "/map";
    aMarker.header.stamp = ros::Time::now();  
    aMarker.type = aMarker.CUBE;
    aMarker.pose.position.x = x;
    aMarker.pose.position.y = y;
    aMarker.pose.position.z = 0;
    aMarker.scale.x = 0.05;
    aMarker.scale.y = 0.05;
    aMarker.scale.z = 0.05;
    aMarker.color.a = 1;
    aMarker.color.g = g;
    aMarker.color.b = b;
    aMarker.id = count;
    aMarker.ns = "sector";
    count++;
    markerArray.markers.push_back(aMarker);
}

//calculate each point's gaussian value.
void calGaussian()
{
    double sigma = 0.7;
    int length = human_list.size();
    for (int i = 0; i < length; i++)
    {
        for(int j = 0; j < length; j++)
        {
            if(i != j && human_list[i].x != human_list[j].x && human_list[i].y != human_list[j].y)
                human_list[i].gaussian += gaussian2D(human_list[i].x, human_list[j].x, human_list[i].y, human_list[j].y, sigma);
        }
    }
}

//filter the points, if smaller than remove.
void filter(std::vector<human> human_list)
{
    double biggest_gaussian = 0.0;
    int length = human_list.size();
    for(int i = 0; i < length ; i++)
    {
        if(human_list[i].gaussian >= biggest_gaussian)
            biggest_gaussian = human_list[i].gaussian;
    }
    for (int i = 0; i < length; ++i)
    {
        if(human_list[i].gaussian > 0.4 * biggest_gaussian)
        {
            new_human_list.push_back(human_list[i]);
            visualization_msgs::Marker aMarker;
            aMarker.header.frame_id = "/map";
            aMarker.header.stamp = ros::Time::now();  
            aMarker.type = aMarker.CUBE;
            aMarker.scale.x = 0.05;
            aMarker.scale.y = 0.05;
            aMarker.scale.z = 0.05;
            aMarker.color.a = 1;
            aMarker.color.g = 1;
            aMarker.id = count_;
            aMarker.ns = "sector";
            aMarker.pose.position.x = human_list[i].x;
            aMarker.pose.position.y = human_list[i].y;
            aMarker.pose.position.z = 0;
            count_++;
            newPoseMarker.markers.push_back(aMarker);
        }
    }


}

void poseCallBack(const geometry_msgs::PoseConstPtr &msg)
{
    ROS_INFO("get robot position");

    visualization_msgs::Marker aMarker;
    aMarker.header.frame_id = "/map";
    aMarker.header.stamp = ros::Time::now();  
    aMarker.type = aMarker.CUBE;
    aMarker.pose.position.x = msg->position.x;
    aMarker.pose.position.y = msg->position.y;
    aMarker.pose.position.z = 0;
    aMarker.scale.x = 0.05;
    aMarker.scale.y = 0.05;
    aMarker.scale.z = 0.05;
    aMarker.color.a = 1;
    aMarker.color.g = 0;
    aMarker.color.b = 1;
    aMarker.id = count1;
    aMarker.ns = "sector";
    count1++;
    robotPoseMarker.markers.push_back(aMarker);
}

void mapCallBack(const nav_msgs::OccupancyGridConstPtr &msg)
{
    map = *msg;
    map_width = map.info.width;
    map_height = map.info.height;
    map_resolution = map.info.resolution;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pose_pub");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    ros::Subscriber map_sub = n.subscribe("/map",1,mapCallBack);
    ros::Subscriber pose_sub = n.subscribe("/robot_pose",1,poseCallBack);
    ros::Publisher pose_pub_marker = n.advertise<visualization_msgs::MarkerArray>("/human_position", 1);
    ros::Publisher new_pose_pub_marker = n.advertise<visualization_msgs::MarkerArray>("/new_human_position", 1);
    ros::Publisher robot_pose_pub_marker = n.advertise<visualization_msgs::MarkerArray>("/robot_position", 1);

    //read txt file and get the data.
    std::ifstream inf;
    std::string line;
    inf.open("/home/lin/catkin_ws/src/detect_human/result/testposition.txt", std::ifstream::in);
    
    int j = 0;
    size_t comma = 0;
    size_t comma1 = 0;
    size_t comma2 = 0; 
    size_t comma3 = 0; 
    size_t comma4 = 0; 

    while (!inf.eof())
    {
        human newHuman;

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

        double x = 0; 
        double y = 0;

        getline(inf,line);
        //get x
        comma = line.find(',',0);
        double comma_ = std::atof(line.substr(0,comma).c_str());
        x = comma_;
        newHuman.x = comma_;
        aMarker.pose.position.x = comma_;
        //get y 
        while (comma < line.size() && j == 0)
        {
            comma1 = line.find(',',comma + 1);
            double comma1_ = std::atof(line.substr(comma + 1,comma1-comma-1).c_str());
            y = comma1_;
            aMarker.pose.position.y = comma1_;
            newHuman.y = comma1_;
            ++j;
            comma = comma1;
        }
        //get z
        while (comma < line.size() && j == 1)
        {
            comma2 = line.find(',',comma + 1);
            double comma2_ = std::atof(line.substr(comma + 1,comma2-comma-1).c_str());
            aMarker.pose.position.z = 0;
            ++j;
            comma = comma2;
        }
        //get sec
        while (comma < line.size() && j == 2)
        {
            comma3 = line.find(',',comma + 1);
            double comma3_ = std::atof(line.substr(comma + 1,comma3-comma-1).c_str());
            newHuman.sec = comma3_;
            ++j;
            comma = comma3;
        }
        //get nsec
        while (comma < line.size() && j == 3)
        {
            comma4 = line.find(',',comma + 1);
            double comma4_ = std::atof(line.substr(comma + 1,comma4-comma-1).c_str());
            newHuman.nsec = comma4_;
            ++j;
            comma = comma4;
        }
        j = 0;
        count++;
        newHuman.gaussian = 0.0;
        human_list.push_back(newHuman);
        poseMarker.markers.push_back(aMarker);    
    }
 
    inf.close();

    //calculate each point's gaussian value
    calGaussian();
    //filter the points
    filter(human_list);

    ROS_INFO("old lengh %d", human_list.size());
    ROS_INFO("new lengh %d", new_human_list.size());
    while(ros::ok())
    {
        pose_pub_marker.publish(poseMarker);
        new_pose_pub_marker.publish(newPoseMarker);
        robot_pose_pub_marker.publish(robotPoseMarker);
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
     
    return 0;
} 
