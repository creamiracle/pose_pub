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
    int clusterNo;
    int old_index;
}human;

typedef struct Point{
    double x;
    double y;
};

typedef struct map_grid
{
    int coords[2];
    double human_dendity;
}maps;

visualization_msgs::MarkerArray poseMarker;
visualization_msgs::MarkerArray newPoseMarker;
visualization_msgs::MarkerArray robotPoseMarker;
visualization_msgs::MarkerArray clusterMarker;
visualization_msgs::MarkerArray lineMarker;


nav_msgs::OccupancyGrid map;

std::vector<human> human_list;
std::vector<human> new_human_list;
std::vector<geometry_msgs::Pose> robotPose;
std::vector<Point> points;

int count = 0;
int count_ = 0;
int count1 = 0;
int clusterCount = 0;
int map_width = 0;
int map_height = 0;
float map_resolution = 0.0;
double threshold = 0.325;

double findBiggestX(std::vector<human> templateVec)
{
    double biggestX = templateVec[1].x;
    for(int i = 0 ; i < templateVec.size(); i++)
    {
        if (biggestX < templateVec[i].x)
        {
            biggestX = templateVec[i].x;
        }
    }
    return biggestX;
}

double findBiggestY(std::vector<human> templateVec)
{
    double biggestY =  templateVec[1].y;
    for(int i = 0 ; i < templateVec.size(); i++)
    {
        if (biggestY < templateVec[i].y)
        {
            biggestY = templateVec[i].y;
        }
    }
    return biggestY;
}

double findSmallestX(std::vector<human> templateVec)
{
    double smallestX =  templateVec[1].x;
    for(int i = 0 ; i < templateVec.size(); i++)
    {
        if (smallestX > templateVec[i].x)
        {
            smallestX = templateVec[i].x;
        }
    }
    return smallestX;
}

double findSmallestY(std::vector<human> templateVec)
{
    double smallestY =  templateVec[1].y;
    for(int i = 0 ; i < templateVec.size(); i++)
    {
        if (smallestY > templateVec[i].y)
        {
            smallestY = templateVec[i].y;
        }
    }
    return smallestY;
}

static const inline double distance(double x1, double y1, double x2, double y2)
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    
    return std::sqrt(dx * dx + dy * dy);
}

const inline int region_query(const std::vector<Point> &input, int p, std::vector<int> &output, double eps)
{
    for(int i = 0; i < (int)input.size(); i++){
        
        if(distance(input[i].x, input[i].y, input[p].x, input[p].y) < eps){
            output.push_back(i);
        }
    }
    
    return output.size();
}

bool expand_cluster(const std::vector<Point> &input, int p, std::vector<int> &output, int cluster, double eps, int min)
{
    std::vector<int> seeds;
    
    if(region_query(input, p, seeds, eps) < min){
        
        //this point is noise
        output[p] = -1;
        return false;
        
    }else{
        
        //set cluster id
        for(int i = 0; i < (int)seeds.size(); i++){
            output[seeds[i]] = cluster;
        }
        
        //delete paint from seeds
        seeds.erase(std::remove(seeds.begin(), seeds.end(), p), seeds.end());
        
        //seed -> empty
        while((int)seeds.size() > 0){
            
            int cp = seeds.front();
            std::vector<int> result;
            
            if(region_query(input, cp, result, eps) >= min){
                
                for(int i = 0; i < (int)result.size(); i++){
                    
                    int rp = result[i];
                    
                    //this paint is noise or unmarked point
                    if(output[rp] < 1){
                        
                        //unmarked point
                        if(!output[rp]){
                            seeds.push_back(rp);
                        }
                        
                        //set cluster id
                        output[rp] = cluster;
                    }
                }
            }
            
            //delete point from seeds
            seeds.erase(std::remove(seeds.begin(), seeds.end(), cp), seeds.end());
        }
    }
    
    return true;
}

int dbscan(const std::vector<Point> &input, std::vector<int> &labels, double eps, int min)
{
    int size = input.size();
    int cluster = 1;
    
    std::vector<int> state(size);
    
    for(int i = 0; i < size; i++){
        
        if(!state[i]){
            
            if(expand_cluster(input, i, state, cluster, eps, min)){
                cluster++;
            }
        }
    }
    
    labels = state;
    
    return cluster - 1;
}

//calculate gaussian value
double gaussian2D(double x, double xc, double y, double yc, double sigma)
{
    double exponent = (std::pow(x - xc, 2) + std::pow(y - yc, 2)) / (2 * std::pow(sigma, 2));
    return std::exp(-exponent);
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

visualization_msgs::Marker genTextMarker(double x, double y, int id, int markerid)
{
    visualization_msgs::Marker aMarker;
    aMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    std::stringstream ss;
    ss<<"Cluster "<<id;
    aMarker.text = ss.str().data();
    aMarker.id = markerid;
    aMarker.header.frame_id = "/map";
    aMarker.header.stamp = ros::Time::now();  
    aMarker.pose.position.x = x;
    aMarker.pose.position.y = y;
    aMarker.pose.position.z = 0.5;
    aMarker.scale.x = 0.25;
    aMarker.scale.y = 0.25;
    aMarker.scale.z = 0.25;
    aMarker.color.a = 1;
    aMarker.color.r = 1.0;
    aMarker.color.g = 0.0;
    aMarker.color.b = 0.0;
    return aMarker;
}

void genMarker(double x, double y, double r, double g, double b)
{
    visualization_msgs::Marker aMarker;
    aMarker.header.frame_id = "/map";
    aMarker.header.stamp = ros::Time::now();  
    aMarker.type = aMarker.CUBE;
    aMarker.scale.x = 0.05;
    aMarker.scale.y = 0.05;
    aMarker.scale.z = 0.05;
    aMarker.color.a = 1;
    aMarker.color.r = r;
    aMarker.color.g = g;
    aMarker.color.b = b;
    aMarker.id = clusterCount;
    aMarker.ns = "sector";
    aMarker.pose.position.x = x;
    aMarker.pose.position.y = y;
    aMarker.pose.position.z = 0;
    clusterCount++;
    clusterMarker.markers.push_back(aMarker);
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
    ROS_INFO("biggest is %f",biggest_gaussian);
    for (int i = 0; i < length; ++i)
    {
        if(human_list[i].gaussian > threshold * biggest_gaussian)
        {
            //save in human list
            human_list[i].old_index = i;
            new_human_list.push_back(human_list[i]);

            //save in point for dbscan
            Point newPoint;
            newPoint.x = human_list[i].x;
            newPoint.y = human_list[i].y;
            points.push_back(newPoint);
            //ROS_INFO("points number %d", points.size());

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
    //ros::Subscriber pose_sub = n.subscribe("/robot_pose",1,poseCallBack);
    ros::Publisher pose_pub_marker = n.advertise<visualization_msgs::MarkerArray>("/human_position", 1);
    ros::Publisher new_pose_pub_marker = n.advertise<visualization_msgs::MarkerArray>("/new_human_position", 1);
    ros::Publisher cluster_pub_marker = n.advertise<visualization_msgs::MarkerArray>("/cluster_position", 1);
    ros::Publisher region_pub_marker = n.advertise<visualization_msgs::Marker>("/region_boundary", 1);
    ros::Publisher text_pub_marker = n.advertise<visualization_msgs::Marker>("/region_boundary_text", 1);

    //read txt file and get the data.
    std::ifstream inf;
    std::string line;
    inf.open("/home/lin/catkin_ws/src/detect_human/result/positions.txt", std::ifstream::in);
    
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
        comma = line.find(' ',0);
        double comma_ = std::atof(line.substr(0,comma).c_str());
        x = comma_;
        newHuman.x = comma_;
        aMarker.pose.position.x = comma_;
        //get y 
        while (comma < line.size() && j == 0)
        {
            comma1 = line.find(' ',comma + 1);
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
            comma2 = line.find(' ',comma + 1);
            double comma2_ = std::atof(line.substr(comma + 1,comma2-comma-1).c_str());
            aMarker.pose.position.z = 0;
            ++j;
            comma = comma2;
        }
        // //get sec
        // while (comma < line.size() && j == 2)
        // {
        //     comma3 = line.find(' ',comma + 1);
        //     double comma3_ = std::atof(line.substr(comma + 1,comma3-comma-1).c_str());
        //     newHuman.sec = comma3_;
        //     ++j;
        //     comma = comma3;
        // }
        // //get nsec
        // while (comma < line.size() && j == 3)
        // {
        //     comma4 = line.find(' ',comma + 1);
        //     double comma4_ = std::atof(line.substr(comma + 1,comma4-comma-1).c_str());
        //     newHuman.nsec = comma4_;
        //     ++j;
        //     comma = comma4;
        // }
        //ROS_INFO("get postion %f, %f",newHuman.x, newHuman.y);
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

    //each label match a point
    std::vector<int> labels;

    //dbscan
    int clusterNum = dbscan(points, labels, 0.4, 3);
    ROS_INFO("number of cluster is %d", clusterNum);

    //give labels to each point in new_human_list
    for (int i = 0; i < labels.size(); ++i)
    {
        new_human_list[i].clusterNo = labels[i];
        //ROS_INFO("point %d is %f %f, label is %d", i,new_human_list[i].x,new_human_list[i].y, new_human_list[i].clusterNo);
        if(labels[i] == -1)
        {
            genMarker(new_human_list[i].x,new_human_list[i].y, 1, 1, 0);
            //ROS_INFO("point %d is %f %f, label is %d", i,new_human_list[i].x,new_human_list[i].y, new_human_list[i].clusterNo);
        }
        else
        {
            genMarker(new_human_list[i].x,new_human_list[i].y, 0.1* labels[i], 0.2* labels[i], 0.1* labels[i]);
        }
    } 
    std::ofstream outfile("/home/lin/catkin_ws/src/pose_pub/result/result20131114.txt");
  
    //ROS_INFO("%f,%f",new_human_list[1].x,new_human_list[1].y);
    for (int j = 1; j <= clusterNum; ++j)
    {
        for (int i = 0; i < new_human_list.size(); ++i)
        {
           if(new_human_list[i].clusterNo == j)
           {
                std::ofstream outfile("/home/lin/catkin_ws/src/pose_pub/result/result20131114.txt", std::ios_base::app);
                if(!outfile)
                {
                  std::cout<<"error";
                }
                else
                {
                  outfile <<"corrdinate is " << new_human_list[i].x << "," << new_human_list[i].y << "," << " cluster No " << new_human_list[i].clusterNo << " , image No is " << new_human_list[i].old_index << std::endl;
                }
            }
        }

    }
    
    while(ros::ok())
    {
        pose_pub_marker.publish(poseMarker);
        new_pose_pub_marker.publish(newPoseMarker);
        cluster_pub_marker.publish(clusterMarker);
        //region_pub_marker.publish(lineMarker);
        int line_counter = 1;
        for (int i = 1; i <= clusterNum; ++i)
        {
            
            //ROS_INFO("cluster no %d",i);
            //ROS_INFO("cluster no %d",line_counter);
            visualization_msgs::Marker line_strip;
            line_strip.header.frame_id = "/map";
            line_strip.header.stamp = ros::Time::now();
            line_strip.ns = "points_and_lines";
            line_strip.action = visualization_msgs::Marker::ADD;
            line_strip.type = visualization_msgs::Marker::LINE_STRIP;
            line_strip.scale.x = 0.1;
            line_strip.color.b = 1.0;
            line_strip.color.a = 1.0;
            line_strip.id = line_counter;

            std::vector<human> templateVec;

            for(int j = 0; j < new_human_list.size(); j++)
            {
                if(new_human_list[j].clusterNo == i)
                {
                        //find all the point with same cluster No
                    templateVec.push_back(new_human_list[j]);
                }
            }
            //ROS_INFO("tme size %d", templateVec.size());
            double biggestX = findBiggestX(templateVec);  
            double biggestY = findBiggestY(templateVec);
            double smallestX = findSmallestX(templateVec);
            double smallestY = findSmallestY(templateVec);
            //ROS_INFO("bigx : %f, bigy : %f, smax : %f , smay : %f",biggestX,biggestY,smallestX,smallestY);
            geometry_msgs::Point p1;
            geometry_msgs::Point p2;
            geometry_msgs::Point p3;
            geometry_msgs::Point p4;

            p1.x = smallestX;
            p1.y = smallestY;
            p1.z = 0;
            line_strip.points.push_back(p1);

            p3.x = biggestX;
            p3.y = smallestY;
            p3.z = 0;
            line_strip.points.push_back(p3);

            p4.x = biggestX;
            p4.y = biggestY;
            p4.z = 0;
            line_strip.points.push_back(p4);
        
            p2.x = smallestX;
            p2.y = biggestY;
            p2.z = 0;
            line_strip.points.push_back(p2);


            line_strip.points.push_back(p1);

            double midX = biggestX - (biggestX - smallestX) / 2;
            double midY = biggestY - (biggestY - smallestY) / 2;
            visualization_msgs::Marker marker =  genTextMarker(midX, midY, i, line_counter);
            region_pub_marker.publish(line_strip);
            text_pub_marker.publish(marker);
            line_counter++;

        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
     
    return 0;
} 
