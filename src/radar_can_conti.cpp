//
// Created by Hardliu on 2020-11-21.
//
#include "radar_can_conti.h"
#include <math.h>

//无参构造函数，对can的初始化参数进行配置
RadarCan_Conti::RadarCan_Conti()
{
    ;
}
//父类的纯虚函数的重写
bool RadarCan_Conti::checkInfo() //
{
    bool flag = true;
    return flag;
}


//父类的纯虚函数的重写，初始化操作，读取参数。
void RadarCan_Conti::setup()
{
    node_handle_.param("/recv_info_freq", recv_info_freq_, 35);//读取launch文件参数，确定接收报文的帧数
    //ROS_INFO("recv_info_freq_ is : %d", recv_info_freq_);
    node_handle_.param("/max_missing_times", max_missing_times_, 100);//读取launch文件参数，确定接收报文的帧数

    node_handle_.param("/conti_radar/long_offset_mid", long_offset_mid_, 0.0);//读取launch文件参数，给定中间雷达在维度上的偏移量 3.6
    node_handle_.param("/conti_radar/lat_offset_mid", lat_offset_mid_, 0.0);//读取launch文件参数，给定中间雷达在经度上的偏移量 0.23
    node_handle_.param("/conti_radar/angular_deviation_id", angular_deviation_mid_, 0.0);//读取launch文件参数，给定中间雷达在经度上的偏移量 0.23

    node_handle_.param("/check_msg_freq", check_msg_freq_, 10);
    node_handle_.param("/send_req_vehicle_msg_freq", send_req_vehicle_msg_freq_, 50);

    //角度转换成为弧度
    angular_deviation_mid_ = angular_deviation_mid_ / 180 * M_PI;
    
    object_mid_num_ = -1;
    //信息容器
    general_info_arr_mid_.clear();
    quality_info_arr_mid_.clear();
    extended_info_arr_mid_.clear();

    
}





void RadarCan_Conti::midMessagePub()
{
//    ROS_INFO("general: %d",general_info_arr_mid_.size());
//    ROS_INFO("quality: %d",quality_info_arr_mid_.size());
//    ROS_INFO("extended: %d",extended_info_arr_mid_.size());

    if(object_mid_num_ == -1)//if not reassign the object_num_ in OBJECT_LIST_STATUS branch
    {
        general_info_arr_mid_.clear();
        quality_info_arr_mid_.clear();
        extended_info_arr_mid_.clear();
        ROS_ERROR("error num of mid object");
    }
    else if(object_mid_num_ >= 0)
    {
        if(object_mid_num_ == general_info_arr_mid_.size() &&
           object_mid_num_ == quality_info_arr_mid_.size() &&
           object_mid_num_ == extended_info_arr_mid_.size())  //in a period recv all the CAN frame
        {
            conti_radar_msgs::conti_Object object_;
            conti_radar_msgs::conti_Objects objects_msgs_;

            for(size_t i = 0; i < general_info_arr_mid_.size(); ++i)
            {
                object_.ID = general_info_arr_mid_[i].object_id_ + 200;//目标ID（从目标被检测到，此ID一直保持不变）
                object_.Object_DistLong = general_info_arr_mid_[i].object_dist_long_;//目标的纵向x坐标
                object_.Object_DistLat = general_info_arr_mid_[i].object_dist_lat_;//目标的横向y坐标
                object_.Object_VrelLong = general_info_arr_mid_[i].object_vre_long_;//目标在纵坐标方向的相对速度x
                object_.Object_VrelLat = general_info_arr_mid_[i].object_vre_lat_;//目标在横坐标方向的相对速度y
                //目标的动态属性，指示对象是否移动或静止（只有给正确的速度和偏航角速度，此值才准确
                object_.Object_DynProp = general_info_arr_mid_[i].object_dyn_prop_;
                object_.Object_RCS = general_info_arr_mid_[i].object_rcs_;//雷带散射截面
                
                for(size_t j = 0; j < quality_info_arr_mid_.size(); ++j)
                {
                    if(object_.ID == quality_info_arr_mid_[j].object_id_ + 200)
                    {
                        object_.Obj_ProbOfExist = quality_info_arr_mid_[j].object_prob_of_exist_;//表示存在的概率
                    }
                }
                for(size_t k = 0; k < extended_info_arr_mid_.size(); ++k)
                {
                    if(object_.ID == extended_info_arr_mid_[k].object_id_ + 200)
                    {
                        object_.Object_OrientationAngel = extended_info_arr_mid_[k].object_orientation_angel_;//目标的方位角
                        object_.Object_Length = extended_info_arr_mid_[k].object_length_;//目标的长度
                        object_.Object_Width = extended_info_arr_mid_[k].object_width_;//目标的宽度
                    }
                }
                objects_msgs_.objects.push_back(object_);//把所有的目标放在目标集中
            }
            objects_msgs_.header.frame_id = "conti_radar_mid";//定义目标集的名字
            objects_msgs_.header.stamp = ros::Time::now();//时间戳
            mid_radar_info_pub_.publish(objects_msgs_);//发布出去

            general_info_arr_mid_.clear();
            quality_info_arr_mid_.clear();
            extended_info_arr_mid_.clear();

            object_mid_num_ = -1;
        }
    }
}

//将接受到的CAN帧数据按照ID进行分类处理
void RadarCan_Conti::transformCanToInfo(std::vector<VCI_CAN_OBJ>& can_frames, int len)
{
    for(int i = 0; i < len; i++)
    {
        int current_id = can_frames[i].ID;
        switch (current_id)
        {
            case RADAR_STATE_MID://0x201 此报文包含radar的当前状态信息
            {
                radar_state_.unPackBytes(can_frames[i]);
                ROS_ERROR("getMaxDistanceCfg:%d",radar_state_.getMaxDistanceCfg());
                ROS_ERROR("getErrorState:%d",radar_state_.getErrorState());
                ROS_ERROR("getSensorID:%d",radar_state_.getSensorID());
                ROS_ERROR("getSortIndex:%d",radar_state_.getSortIndex());
                ROS_ERROR("getRadarPower:%d",radar_state_.getRadarPower());
                ROS_ERROR("getOutputType:%d",radar_state_.getOutputType());
                ROS_ERROR("getSendQuality:%d",radar_state_.getSendQuality());
                ROS_ERROR("getSendExtInfo:%d",radar_state_.getSendExtInfo());
                ROS_ERROR("getMotionRxState:%d",radar_state_.getMotionRxState());
                ROS_ERROR("getRCS_Threshold:%d",radar_state_.getRCS_Threshold());
            }
            case OBJECT_LIST_STATUS_MID://0x60A 此CAN报文中包含有检测到的目标数量信息
            {
                object_list_status_.unPackBytes(can_frames[i]);
                object_mid_num_= object_list_status_.getNumOfObj();
                general_info_arr_mid_.clear();
                quality_info_arr_mid_.clear();
                extended_info_arr_mid_.clear();

                ROS_INFO("I CAN detect %d objects in mid radar",object_mid_num_);

            }
                break;
            case OBJECT_GENERAL_INFO_MID: //0x60B 此CAN报文中包含有检测到的目标ID、目标横纵向坐标，在横纵向的相对速度、
            {
                object_general_info_.unPackBytes(can_frames[i]);
                GeneralInfo general_info_;
                general_info_ = object_general_info_.getGeneralInfo();
                general_info_arr_mid_.emplace_back(general_info_);
            }
                break;
            case OBJECT_QUALITY_INFO_MID://0x60C
            {
                object_quality_info_.unPackBytes(can_frames[i]);
                QualityInfo quality_info_;
                quality_info_ = object_quality_info_.getQualityInfo();
                quality_info_arr_mid_.emplace_back(quality_info_);
            }
                break;
            case OBJECT_EXTENDED_INFO_MID://0x60D
            {
                object_extended_info_.unPackBytes(can_frames[i]);
                ExtendedInfo extended_info_;
                extended_info_ = object_extended_info_.getExtendedInfo();
                extended_info_arr_mid_.emplace_back(extended_info_);
            }
                break;
            default:
                continue;
        }
        midMessagePub();   // 在里面判断这一组是否凑齐。凑齐了就发送。
       
       
    }

}




