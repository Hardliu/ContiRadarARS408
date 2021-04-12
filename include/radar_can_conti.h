//
// Created by Hardliu on 2020-11-21.
//

#ifndef RADAR_CAN_CONTI_H
#define RADAR_CAN_CONTI_H

#include "radar_can.h"
#include <iostream>

#include <conti_radar/radar_state_201.h>
#include <conti_radar/object_list_status_60a.h>
#include <conti_radar/object_general_info_60b.h>
#include <conti_radar/object_quality_info_60c.h>
#include <conti_radar/object_extended_info_60d.h>

#include <conti_radar_msgs/conti_Objects.h>
#include <conti_radar_msgs/conti_Object.h>

const int OBJECT_LIST_STATUS_MID = 0x0000060A;
const int OBJECT_GENERAL_INFO_MID = 0x0000060B;
const int OBJECT_QUALITY_INFO_MID = 0x0000060C;
const int OBJECT_EXTENDED_INFO_MID = 0x0000060D;

const int RADAR_STATE_MID = 0x00000201;


class RadarCan_Conti:public RadarCan //以public方式继承RadarCan父类
{
public:
    RadarCan_Conti(); //无参数的构造函数

    virtual bool checkInfo(); //对父类纯虚函数进行重写
    virtual void setup();//对父类纯虚函数进行重写
    virtual void transformCanToInfo(std::vector<VCI_CAN_OBJ>& can_frames, int len); //对父类纯虚函数进行重写

    void midMessagePub(); 
    


private:

    ObjectListStatus60A object_list_status_;
    ObjectGeneralInfo60B object_general_info_;
    ObjectQualityInfo60C object_quality_info_;
    ObjectExtendedInfo60D object_extended_info_;
    RadarState201 radar_state_;

    std::vector<GeneralInfo> general_info_arr_mid_;
    std::vector<QualityInfo> quality_info_arr_mid_;
    std::vector<ExtendedInfo> extended_info_arr_mid_;

    
    int object_mid_num_;
    

//Used to transform the sensor coordinate system to the vehicle coordinate system.
    double long_offset_mid_;
    double lat_offset_mid_;
    double angular_deviation_mid_;

    

};

#endif
