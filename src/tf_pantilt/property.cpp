#include"mynteye_pointcloud/tf_pantilt.h"

void BroadCaster::__setLaunchParam(){
    ros::NodeHandle n("~");
    n.getParam("FRAME/CAMERA_STAY_LINK",      CAMERA_STAY_LINK);
    n.getParam("FRAME/PAN_LINK",     PAN_LINK);
    n.getParam("FRAME/TILTLINK",         TILTLINK);
}
