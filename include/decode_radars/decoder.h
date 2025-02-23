#ifndef _DECODER_DECODER_H
#define _DECODER_DECODER_H


#include "can_msgs/Frame.h"
#include <iostream>


#include <sensor_msgs/PointCloud2.h>



#include "decode_radars/utilities.h"


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


union r_Point
{

    float data[4];
    
    struct
    {
        float x;
        float y;
        float vrel;
        float rcs;
    }; 
};

class decoder{
    public:


    int mode;
    int id;

    uint count;



    decode_radars::ContiRadarList objects_list;

    std::shared_ptr<
    std::vector
    <pcl::PointXYZI, Eigen::aligned_allocator
    <pcl::PointXYZI>>> radar_points;


    ros::NodeHandle nh;
    
    ros::Subscriber status_can_sub;
    ros::Subscriber content_can_sub;

    ros::Publisher status_decode_pub;
    ros::Publisher content_decode_pub;
    ros::Publisher radarpointcloud_pub;
    ros::Publisher object_list_pub;
    
    ros::Timer timer;



    decoder(int id_, int mode_);

    void status_callback(const can_msgs::Frame& frame);
    void content_callback(const can_msgs::Frame& frame);
    void timer_callback(const ros::TimerEvent& event);


};

#endif