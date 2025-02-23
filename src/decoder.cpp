
#include "decode_radars/decoder.h"
#include <string>

#include <boost/program_options.hpp>





decoder::decoder(int id_, int mode_){
    
    id=id_;
    mode=mode_;

    count=0;

    std::string pub_topic_status="/list_messages";
    std::string pub_topic_content="/list_contents";
    
    objects_list.objects.resize(100);
    


    status_can_sub=nh.subscribe("/can_tx",100,
                                &decoder::status_callback,this);
    content_can_sub=nh.subscribe("/can_tx",100,
                                &decoder::content_callback,this);

    radar_points=std::make_shared<
                            std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI>>
                            >(250);    
    
    if ( mode == _cluster)
    
    
    {




        std::string domain="_cluster_";
        
        std::string topic_status_cluster=pub_topic_status+domain+
                                         std::to_string(id);
        status_decode_pub=nh.advertise<decode_radars::ClusterList>(
            topic_status_cluster, 10
        );

        std::string topic_content_cluster=pub_topic_content+domain+
                                          std::to_string(id);

        content_decode_pub=nh.advertise<decode_radars::ClusterRadar>(
            topic_content_cluster, 10
        );

        std::string pointcloud_topic="/radar/pointcloud_"+std::to_string(id);

    }

    else if (mode == _object)
    {
        std::string domain="_object_";

        std::string topic_status_object=pub_topic_status+domain+
                                         std::to_string(id);

        status_decode_pub=nh.advertise<decode_radars::ContiList>(
            topic_status_object, 50
        );

        std::string topic_content_object=pub_topic_content+domain+
                                          std::to_string(id);

        content_decode_pub=nh.advertise<decode_radars::ContiRadar>(
            topic_content_object, 50
        );

        object_list_pub=nh.advertise<decode_radars::ContiRadarList>(
            topic_content_object+"_list", 100
        );
        

    }


    else{
        std::cerr<<"Error! please choose either\
                    of the available modes"<<std::endl;
    }

}








void decoder::status_callback(const can_msgs::Frame& frame){
    
    int id_status_msg;


    

    // this is for the cluster mode
    if (mode == _cluster){

        

        id_status_msg=1536 + id*16;

        if (frame.id != id_status_msg) return;

        decode_radars::ClusterList output;

        output.nof_targetsnear=frame.data[0];
        output.nof_targetsfar=frame.data[1];

        Byte b2,b3;


        b2.byte=frame.data[2];
        b3.byte=frame.data[3];


        output.meas_counter= b3.bit1*pow(2,0)+b3.bit2*pow(2,1)+
                             b3.bit3*pow(2,2)+b3.bit4*pow(2,3)+
                             b3.bit5*pow(2,4)+b3.bit6*pow(2,5)+
                             b3.bit7*pow(2,6)+b3.bit8*pow(2,7) +
                             
                             b2.bit1*pow(2,8)+
                             b2.bit2*pow(2,9) +b2.bit3*pow(2,10)+
                             b2.bit4*pow(2,11)+b2.bit5*pow(2,12)+
                             b2.bit6*pow(2,13)+b2.bit7*pow(2,14)+
                             b2.bit8*pow(2,15);
        
        output.interface_version=frame.data[4];



        status_decode_pub.publish(output);

    }



    // this is for the object mode
    else if (mode == _object){

        
        id_status_msg=1546 + id*16;

        if (frame.id != id_status_msg) return;

        decode_radars::ContiList output;

        output.nof_objects=frame.data[0];

        Byte b1,b2;

        b1.byte=frame.data[1];
        b2.byte=frame.data[2];

        output.meas_counter= b2.bit1*pow(2,0)+b2.bit2*pow(2,1)+
                             b2.bit3*pow(2,2)+b2.bit4*pow(2,3)+
                             b2.bit5*pow(2,4)+b2.bit6*pow(2,5)+
                             b2.bit7*pow(2,6)+b2.bit8*pow(2,7) +
                             
                             b1.bit1*pow(2,8)+
                             b1.bit2*pow(2,9) +b1.bit3*pow(2,10)+
                             b1.bit4*pow(2,11)+b1.bit5*pow(2,12)+
                             b1.bit6*pow(2,13)+b1.bit7*pow(2,14)+
                             b1.bit8*pow(2,15);

        output.interface_version=frame.data[3];





        status_decode_pub.publish(output);
    }



    else{
        std::cerr<<"none of the modes activated"<<std::endl;
        return;
    }



}


void decoder::content_callback(const can_msgs::Frame& frame){

    int id_content_msg;


     if (mode==_cluster){
        
        

        id_content_msg=1793 + id*16;

        if (frame.id!=id_content_msg) return;
        
        decode_radars::ClusterRadar output;

        output.target_id=frame.data[0];

        Byte b1,b2,b3,b4,b5,b6;

        b1.byte=frame.data[1];
        b2.byte=frame.data[2];
        b3.byte=frame.data[3];
        b4.byte=frame.data[4];
        b5.byte=frame.data[5];
        b6.byte=frame.data[6];

        output.longitude_dist=TARGET_DIST_LONG_MIN+ TARGET_DIST_RES*(
            pow(2,0)*b2.bit4 + pow(2,1)*b2.bit5 + pow(2,2)*b2.bit6 +
            pow(2,3)*b2.bit7 + pow(2,4)*b2.bit8 +
            pow(2,5)*b1.bit1 + pow(2,6)*b1.bit2 + pow(2,7)*b1.bit3 + 
            pow(2,8)*b1.bit4 + pow(2,9)*b1.bit5 +
            pow(2,10)*b1.bit6 + pow(2,11)*b1.bit7 + pow(2,12)*b1.bit8          
        );

        output.lateral_dist=TARGET_DIST_LAT_MIN + TARGET_DIST_RES*(
            pow(2,0)*b3.bit1+pow(2,1)*b3.bit2+
            pow(2,2)*b3.bit3+pow(2,3)*b3.bit4+
            pow(2,4)*b3.bit5+pow(2,5)*b3.bit6+pow(2,6)*b3.bit7+
            pow(2,7)*b3.bit8+pow(2,8)*b2.bit1+pow(2,9)*b2.bit2);
        
        output.longitude_vel=TARGET_VREL_LONG_MIN + TARGET_VREL_RES*(
            pow(2,0)*b5.bit7+pow(2,1)*b5.bit8+pow(2,2)*b4.bit1+
            pow(2,3)*b4.bit2+pow(2,4)*b4.bit3+pow(2,5)*b4.bit4+
            pow(2,6)*b4.bit5+pow(2,7)*b4.bit6+pow(2,8)*b4.bit7+
            pow(2,9)*b4.bit8);

        output.lateral_vel= TARGET_VREL_LAT_MIN + TARGET_VREL_RES*(
            pow(2,0)*b6.bit6+pow(2,1)*b6.bit7+
            pow(2,2)*b6.bit8+pow(2,3)*b5.bit1+pow(2,4)*b5.bit2+
            pow(2,5)*b5.bit3+pow(2,6)*b5.bit4+pow(2,7)*b5.bit5+
            pow(2,8)*b5.bit6);
        
        output.rcs=TARGET_RCS_MIN + TARGET_RCS_RES*(frame.data[7]);
        
        output.header.frame_id="radar_";
        output.header.stamp=frame.header.stamp;

        (*radar_points)[output.target_id].x=output.longitude_dist,
        (*radar_points)[output.target_id].y=output.lateral_dist,
        (*radar_points)[output.target_id].z=output.longitude_vel,
        (*radar_points)[output.target_id].intensity=output.rcs;


    
        content_decode_pub.publish(output);

    }


    else if (mode == _object){
        id_content_msg=1547 + id*16;


        if (frame.id!=id_content_msg) return;

        decode_radars::ContiRadar output;

        output.obstacle_id=frame.data[0];

        Byte b1,b2,b3,b4,b5,b6;

        b1.byte=frame.data[1];
        b2.byte=frame.data[2];
        b3.byte=frame.data[3];
        b4.byte=frame.data[4];
        b5.byte=frame.data[5];
        b6.byte=frame.data[6];

        output.longitude_dist = OBJECT_DIST_LONG_MIN+
        OBJECT_DIST_RES*(pow(2,0)*b2.bit4+pow(2,1)*b2.bit5+
        pow(2,2)*b2.bit6+pow(2,3)*b2.bit7+pow(2,4)*b2.bit8+pow(2,5)*b1.bit1+
        pow(2,6)*b1.bit2+pow(2,7)*b1.bit3+pow(2,8)*b1.bit4+pow(2,9)*b1.bit5+
        pow(2,10)*b1.bit6+pow(2,11)*b1.bit7+pow(2,12)*b1.bit8);
        
        output.lateral_dist = OBJECT_DIST_LAT_MIN+
        OBJECT_DIST_RES*(pow(2,0)*b3.bit1+pow(2,1)*b3.bit2+
        pow(2,2)*b3.bit3+pow(2,3)*b3.bit4+pow(2,4)*b3.bit5+
        pow(2,5)*b3.bit6+pow(2,6)*b3.bit7+pow(2,7)*b3.bit8+
        pow(2,8)*b2.bit1+pow(2,9)*b2.bit2+pow(2,10)*b2.bit3);
        
        output.longitude_vel = OBJECT_VREL_LONG_MIN+
        OBJECT_VREL_RES*(pow(2,0)*b5.bit7+pow(2,1)*b5.bit8+
        pow(2,2)*b4.bit1+pow(2,3)*b4.bit2+pow(2,4)*b4.bit3+
        pow(2,5)*b4.bit4+pow(2,6)*b4.bit5+pow(2,7)*b4.bit6+
        pow(2,8)*b4.bit7+pow(2,9)*b4.bit8);
        
        output.lateral_vel = OBJECT_VREL_LAT_MIN+
        OBJECT_VREL_RES*(pow(2,0)*b6.bit6+pow(2,1)*b6.bit7+
        pow(2,2)*b6.bit8+pow(2,3)*b5.bit1+pow(2,4)*b5.bit2+
        pow(2,5)*b5.bit3+pow(2,6)*b5.bit4+pow(2,7)*b5.bit5+
        pow(2,8)*b5.bit6);
        
        output.rcs=OBJECT_RCS_MIN+OBJECT_RCS_RES*frame.data[7];



        
        output.header.frame_id="radar_front";
        output.header.stamp=frame.header.stamp;


        objects_list.objects[output.obstacle_id]=output;


        content_decode_pub.publish(output);

    }


    else{

        return;

    }

}