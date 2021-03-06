//
// Created by zxkj on 18-12-01.
//

#ifndef CONTI_RADAR_OBJECT_EXTENDED_INFO_60D_H
#define CONTI_RADAR_OBJECT_EXTENDED_INFO_60D_H

#include "controlcan.h"


struct ExtendedInfo  //扩展帧信息
{
    int object_id_;
    float object_arel_long_;
    int object_class_;
    float object_arel_lat_;
    float object_orientation_angel_;
    float object_length_;
    float object_width_;
};
class ObjectExtendedInfo60D
{
private:
    int object_id_;
    float object_arel_long_;
    int object_class_;
    float object_arel_lat_;
    float object_orientation_angel_;
    float object_length_;
    float object_width_;

public:
    ObjectExtendedInfo60D();
    void unPackBytes(const VCI_CAN_OBJ& can_frame);

    int getObjectId();
    float getObjectArelLong();
    int getObjectClass();
    float getObjectArelLat();
    float getObjectOrientationAngel();
    float getObjectLength();
    float getObjectWidth();

    ExtendedInfo getExtendedInfo();
};


#endif //CONTI_RADAR_OBJECT_EXTENDED_INFO_60D_H
