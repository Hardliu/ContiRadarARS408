//
// Created by Hardliu on 2020-11-21.
//
#include "radar_can.h"
#include <boost/thread.hpp>
#include <csignal>
#include <conti_radar_msgs/conti_Objects.h>
#include <conti_radar_msgs/conti_Object.h>


bool RadarCan::is_quit_ = false;//？
int rec_len = 0;
 //类RadarCan的无参构造函数，打开CAN设备并读取CAN设备型号
RadarCan::RadarCan()          
{
    is_recv_info_quit_ = false;//？
    missing_can_info_counter_ = 0;
    missing_gnss_info_counter_ = 0; 
    missing_vehicle_info_counter_ = 0;
    canOpen(); 
}

RadarCan::~RadarCan()//类RadarCan的析构函数
{
    mid_radar_info_pub_.shutdown();
    canClose();
    ros::shutdown();//关闭ros节点
    ROS_INFO("RadarCan ～function!");
}
//定时读取CAN1通道报文数据，并转化为有效信息，此函数是一个定时回调函数
void RadarCan::recvInfoCallback(const ros::TimerEvent&)
{
    VCI_CAN_OBJ buf[100];  // data buffer
    std::vector<VCI_CAN_OBJ> recs; // CAN data frame struct, is related to recv_info_freq_
    if((rec_len = VCI_Receive(VCI_USBCAN2,0,0,buf,100,0))>0)
    {
        
        missing_can_info_counter_ = 0;
        VCI_ClearBuffer(VCI_USBCAN2, 0, 0);//清空指定CAN通道的缓冲区
        for(int i = 0; i < rec_len; i++)
        {
            recs.emplace_back(buf[i]); //调用了转移构造函数，在向容器插入的时候直接构造，效率更高，减少额外空间的开辟
        }
        ROS_INFO("I recv %d CAN frames from radar",rec_len);
        transformCanToInfo(recs,rec_len); //这是RadarCan类里的纯虚函数
    }
    //}
}

//订阅车辆速度信息节点的回调函数：获取当前车速信息
void RadarCan::recvVehicleInfoCallback(const pnc_msgs::VehicleInfo::ConstPtr vehicleinfo)
{
    vehicle_msg_speed_ = vehicleinfo->vehicle_speed;
    missing_vehicle_info_counter_ = 0;
}

//订阅gnss信息节点的回调函数：获取当前的车辆横摆角速度
void RadarCan::recvGnssInfoCallback(const raw_sensor_msgs::GnssImuInfo::ConstPtr gnssimuinfo)
{
    vehicle_msg_yaw_rate_ = gnssimuinfo->yaw_rate; //从Gnss处获得的信息来更新车辆的yaw_rate
    missing_gnss_info_counter_ = 0;
}

//void RadarCan::quit()
//{
//    mid_radar_info_pub_.shutdown();
//    left_radar_info_pub_.shutdown();
//    right_radar_info_pub_.shutdown();
//    canClose();
//
//    ros::shutdown();
//}

//定时检查是否接收到了三个订阅节点的信息，如果没有就会打印信息提醒
void RadarCan::checkMsgCallback(const ros::TimerEvent&)
{
    ++missing_can_info_counter_;
    ++missing_gnss_info_counter_;
    ++missing_vehicle_info_counter_;

    if (missing_can_info_counter_ > 10)
        ROS_ERROR("can't receive radar msgs!");

    if (missing_vehicle_info_counter_ > 10)
        ROS_ERROR("can't recevie vehicle_info msgs!");

    if (missing_gnss_info_counter_ > 10)
        ROS_ERROR("can't receive gnss_imu_info msgs!");

}

//定时通过0x300和0x301 ID向雷达发送报文数据，输入当前的速度和横摆角速度
void RadarCan::sendReqVehiclemsgCallback(const ros::TimerEvent&) //通过订阅车辆的速度信息，向雷达更新车辆当前的速度
{

   /*
    VCI_CAN_OBJ  send[2];
    send[0].ID = 0x300;
    send[1].ID = 0x301;
    
    
    send[0].RemoteFlag = 0;  // 0-数据帧；1-远程帧
    send[0].ExternFlag = 0;  // 0-标准帧；1-扩展帧
    send[0].DataLen = 2;
    send[0].SendType = 0;  // 0-正常发送;1-单次发送;2-自发自收;3-单次自发自收
    send[0].arryData[0] = (uint8_t)(((((uint16_t)(vehicle_msg_speed_ / 3.6 / 0.02)) >> 8) & 0x1F));//此处根据当前挡位状态：+ (vehicle_msg_motion_status_ << 6));
    send[0].arryData[1] = (uint8_t)(((uint16_t)(vehicle_msg_speed_ / 3.6 / 0.02)) & 0x00FF);
    //向雷达发送车速的报文数据
    VCI_Transmit(VCI_USBCAN2, 0,0, &send[0] , 1);
    
    send[1].RemoteFlag = 0;  // 0-数据帧；1-远程帧
    send[1].ExternFlag = 0;  // 0-标准帧；1-扩展帧
    send[1].DataLen = 2;
    send[1].SendType = 0;  // 0-正常发送;1-单次发送;2-自发自收;3-单次自发自收
    send[1].arryData[0] = (uint8_t)((((uint16_t)(vehicle_msg_yaw_rate_ / 0.01 + 32768)) & 0xFF00) >> 8);
    send[1].arryData[1] = (uint8_t)(((uint16_t)(vehicle_msg_yaw_rate_ / 0.01 + 32768)) & 0x00FF);
    //向雷达发送车辆横摆角速度的报文数据
    VCI_Transmit(VCI_USBCAN2, 0,0, &send[1] , 1);
    */
}



//循环监听、发送数据函数
int RadarCan::doListening()
{
   
        canStart(); //初始化CAN 设置500Kbit/s
      

        ROS_INFO("begin to listen!");
        //车速信息节点订阅+回调
        vehicle_info_sub_ = node_handle_.subscribe(
                "pnc_msgs/vehicle_info", 1, &RadarCan::recvVehicleInfoCallback, this);
        //gnss信息节点订阅+回调
        gnss_info_sub_ = node_handle_.subscribe(
                "/trunk_info/gnss_ins", 1, &RadarCan::recvGnssInfoCallback, this);
        //只用中间一个雷达conti_radar_mid，发布objects信息
        mid_radar_info_pub_ = node_handle_.advertise<conti_radar_msgs::conti_Objects>(
                "/conti_radar_1/objects", 1);
        //定义ros定时回调函数-
        recv_info_sub_ = node_handle_.createTimer(
                ros::Duration(1.0 / recv_info_freq_), &RadarCan::recvInfoCallback, this, false, true); //每（1/recv_info_freq_）s触发一次
        //定义ros定时回调函数-检查收到的消息
        check_msg_ = node_handle_.createTimer(
                ros::Duration(1.0 / check_msg_freq_), &RadarCan::checkMsgCallback, this, false, true);//每（1/check_msg_freq_）s触发一次
        //定义ros定时回调函数-向雷达发送车辆速度信息
        send_req_vehicle_msg_ = node_handle_.createTimer(
                ros::Duration(1.0 / send_req_vehicle_msg_freq_), &RadarCan::sendReqVehiclemsgCallback, this, false, true);//每（1/send_req_vehicle_msg_freq_）s触发一次
        //与回调函数搭配
        ros::spin();

        ROS_INFO("stop to listen!");
    
}


//判断CAN设备是否成功打开，并获取CAN设备信息
int RadarCan::canOpen() 
{
    VCI_BOARD_INFO pInfo;
    //此设备的只能打开一次
	if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)//打开设备
	{
		printf(">>open deivce success!\n");//打开设备成功
	}else
	{
		printf(">>open deivce error!\n");
		//exit(1);
	}
	if(VCI_ReadBoardInfo(VCI_USBCAN2,0,&pInfo)==1)//读取设备序列号、版本等信息。
	{
		printf(">>Get VCI_ReadBoardInfo success!\n");
		printf(">>Serial_Num:%c", pInfo.str_Serial_Num[0]);
		printf("%c", pInfo.str_Serial_Num[1]);
		printf("%c", pInfo.str_Serial_Num[2]);
		printf("%c", pInfo.str_Serial_Num[3]);
		printf("%c", pInfo.str_Serial_Num[4]);
		printf("%c", pInfo.str_Serial_Num[5]);
		printf("%c", pInfo.str_Serial_Num[6]);
		printf("%c", pInfo.str_Serial_Num[7]);
		printf("%c", pInfo.str_Serial_Num[8]);
		printf("%c", pInfo.str_Serial_Num[9]);
		printf("%c", pInfo.str_Serial_Num[10]);
        printf("\n");

		printf(">>hw_Type:%c", pInfo.str_hw_Type[0]);
		printf("%c", pInfo.str_hw_Type[1]);
		printf("%c", pInfo.str_hw_Type[2]);
		printf("%c", pInfo.str_hw_Type[3]);
		printf("%c", pInfo.str_hw_Type[4]);
		printf("%c", pInfo.str_hw_Type[5]);
		printf("%c", pInfo.str_hw_Type[6]);
		printf("%c", pInfo.str_hw_Type[7]);
		printf("%c", pInfo.str_hw_Type[8]);
		printf("%c", pInfo.str_hw_Type[9]);printf("\n");	
	}else
	{
		printf(">>Get VCI_ReadBoardInfo error!\n");
		//exit(1);
	}
    //配置雷达的状态，配置一次就可以，雷达具有记忆保存功能
    /*
    VCI_CAN_OBJ send[1];
	send[0].ID=0x00000200;
	send[0].SendType=0;
	send[0].RemoteFlag=0;
	send[0].ExternFlag=0;
	send[0].DataLen=8;
	for(int i = 0;i<send[0].DataLen;i++)
    {
        send[0].arryData[0] = (uint8_t)((uint16_t)(0x88));
        send[0].arryData[1] = (uint8_t)((uint16_t)(0x00));
        send[0].arryData[2] = (uint8_t)((uint16_t)(0x80));
        send[0].arryData[3] = (uint8_t)((uint16_t)(0x00));
        send[0].arryData[4] = (uint8_t)((uint16_t)(0x08));
        send[0].arryData[5] = (uint8_t)((uint16_t)(0x80));
        send[0].arryData[6] = (uint8_t)((uint16_t)(0x00));
        send[0].arryData[7] = (uint8_t)((uint16_t)(0x00));

    }
	if(VCI_Transmit(VCI_USBCAN2, 0, 0, &send[0], 1) == 1)
    {
        printf("radar state configration is success!");
    }
    */
    return 0;
}

//CAN设备初始化：设置波特率、工作模式等，并用VCI_StartCAN函数打开CAN的通道1
int RadarCan::canStart() 
{
    VCI_INIT_CONFIG can_init_config;
    can_init_config.AccCode=0;
	can_init_config.AccMask=0xFFFFFFFF;
	can_init_config.Filter=1;//接收所有帧
	can_init_config.Timing0=0x00;/*波特率500 Kbps  Timing0:0x00  Timing1:0x1C*/
	can_init_config.Timing1=0x1C;
	can_init_config.Mode=0;//正常模式
    
    if(VCI_InitCAN(VCI_USBCAN2,0,0,&can_init_config)!=1)
	{
		printf(">>Init CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);
	}

	if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
	{
		printf(">>Start CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);

	}
    
    return 0;
}


//关闭CAN设备
int RadarCan::canClose()
{
    
    VCI_CloseDevice(VCI_USBCAN2,0);
    ROS_INFO("close device ok!");
}
