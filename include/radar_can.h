//
// Created by Hardliu on 2020-11-21.
//

#ifndef RADAR_CAN_H
#define RADAR_CAN_H

#include "controlcan.h"
#include "ros/ros.h"
#include <iostream>
#include "pnc_msgs/VehicleInfo.h" //车速信息的消息
#include "raw_sensor_msgs/GnssImuInfo.h" //车辆横摆角速度的消息
//#include "pnc_msgs/Gear.h"  车辆挡位信息的消息
#include <thread>



class RadarCan
{
public:
    RadarCan();
    ~RadarCan();

    int doListening(); // 主函数 ：一个一直在监听的函数

//  void recvCmdCallBack();
    void recvInfoCallback(const ros::TimerEvent&); //定时回调函数
    void recvVehicleInfoCallback(const pnc_msgs::VehicleInfo::ConstPtr vehicleinfo);//vehicle_info_sub_的回调函数
    void recvGnssInfoCallback(const raw_sensor_msgs::GnssImuInfo::ConstPtr gnssimuinfo);//gnss_info_sub_的回调函数
    void checkMsgCallback(const ros::TimerEvent&); //定时回调函数
    void sendReqVehiclemsgCallback(const ros::TimerEvent&); //定时回调函数
// 三个纯虚函数，此类不能实例化对象
    virtual void setup() = 0;
    virtual void transformCanToInfo(std::vector<VCI_CAN_OBJ>& can_frames, int len) = 0; //接收读取数组式序列容器形式的的CAN报文和该数组的大小（也即读取到的CAN报文数量）
    virtual bool checkInfo() = 0;

protected:
    //ros interface
    ros::Publisher mid_radar_info_pub_; //创建Publisher mid_radar_info_pub_
    ros::Subscriber vehicle_info_sub_; //创建Subscriber
    ros::Subscriber gnss_info_sub_;//创建Subscriber

    ros::NodeHandle node_handle_;//实例化句柄
   
    ros::Timer recv_info_sub_; //创建一个定时触发回调函数的对象
    ros::Timer check_msg_;//创建一个定时触发回调函数的对象
    ros::Timer send_req_vehicle_msg_;//创建一个定时触发回调函数的对象

    //can interface
    
    int canStart();//配置CAN
    int canOpen();//验证是否打开CAN的函数，获取CAN设备信息
    int canClose();//关闭设备
    
    //int canWrite(CanParameters can_parameters, unsigned int id, unsigned char* buf, unsigned char len);

    int missing_vehicle_info_counter_;
    int missing_gnss_info_counter_;
    int missing_can_info_counter_;

    int time_sharing_counter_ = 0;

    int recv_info_freq_;
    int check_msg_freq_;
    int send_req_vehicle_msg_freq_;
    bool is_recv_info_quit_;

    int max_missing_times_;

    double vehicle_msg_speed_ = 0;//provide by speed
    double vehicle_msg_yaw_rate_ = 0;//provide by gnss_info
    
public:
    static bool is_quit_;
};

#endif //CONTI_RADAR_RADAR_CAN_H
