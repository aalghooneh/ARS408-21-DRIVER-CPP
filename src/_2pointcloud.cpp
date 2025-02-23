// pcl includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// ros includes
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Odometry.h>


// inside includes
#include <decode_radars/ClusterList.h>
#include <decode_radars/ClusterRadar.h>




// STL includes
#include <vector>
#include <iostream>


std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI>> points_;

ros::Subscriber status_sub;
ros::Subscriber content_sub;
ros::Subscriber odom_sub;

ros::Publisher pointcloud_pub;

ros::Publisher velocityMarkerPub;

std::shared_ptr<ros::NodeHandle> nh_ptr;

pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZI>());

uint expected_size=1000;
bool satisfied = true;

void content_callback(const decode_radars::ClusterRadar& radar_msg)
{
    pcl::PointXYZI p; 
    
    uint8_t targetId=radar_msg.target_id;

    p.x=radar_msg.longitude_dist; p.y=radar_msg.lateral_dist; 
    
    float azimuth=atan2(radar_msg.lateral_dist, radar_msg.longitude_dist);

    if (abs(radar_msg.longitude_vel) > 17.5){
        return;
    }
    const double pi = std::acos(-1);
    const double lower_bound = -70 * pi / 180;
    const double upper_bound = 70 * pi / 180;

    if (!(azimuth > lower_bound && azimuth < upper_bound)) {
        return;
    }

    p.z=radar_msg.longitude_vel + linearVelocity * cos(azimuth); p.intensity=radar_msg.rcs;

    if (p.z < 0.5) p.z=0.0f;

    visualization_msgs::Marker velocityMarker;
    velocityMarker.id=radar_msg.target_id;

    markerMaker(velocityMarker, radar_msg.longitude_dist, radar_msg.lateral_dist, p.z);

    velocityMarkerPub.publish(velocityMarker);  



    points_.push_back(p);
    if (points_.size()>expected_size)
    {
        pointcloud->resize(points_.size());
        pointcloud->points=points_;


        //for (auto& point: pointcloud->points) point.z=0.0f;

        sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*pointcloud, *msg);

        msg->header.frame_id="radar_front";
        msg->header.stamp=ros::Time::now();

        pointcloud_pub.publish(msg);
        points_.clear();
        satisfied=true;
    }
}

void init(int id_, int mode_)
{
    points_.reserve(250);

    std::string topic_sub_status="list_messages_cluster_"+std::to_string(id_);
    std::string topic_sub_content="list_contents_cluster_"+std::to_string(id_);

    status_sub=nh_ptr->subscribe(topic_sub_status, 10, &status_callback);
    content_sub=nh_ptr->subscribe("/list_contents_cluster_2", 10, &content_callback);

    odom_sub=nh_ptr->subscribe("/ego_odom", 100, &odomCallback);

    pointcloud_pub=nh_ptr->advertise<sensor_msgs::PointCloud2>("/radar_pointcloud",10);
    velocityMarkerPub=nh_ptr->advertise<visualization_msgs::Marker>("/velocity_marker",10);

    
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud");


    nh_ptr=std::make_shared<ros::NodeHandle>();



    int id=2;
    int mode=0;

    init(id, mode);

    ros::spin();
    return 0;
}
