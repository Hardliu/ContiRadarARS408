//
// Created by zxkj on 18-12-01.
//


#include "radar_can_conti.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "conti_radar_node");

    //多态的技术：父类的指针或者引用指向子类对象
    RadarCan *radar_can = new RadarCan_Conti;

    
    radar_can->setup();
    radar_can->doListening();

    delete radar_can;
    printf("over!");
    return 0;
}
