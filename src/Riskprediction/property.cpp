#include<mynteye_pointcloud/Riskprediction.h>

void RiskClass::__setLaunchParam(){
    ros::NodeHandle n("~");
    n.getParam("FRAME/ROBOT_BASE",      FRAME_ROBOT_BASE);
    n.getParam("FRAME/CAMERA_BASE",     FRAME_CAMERA_BASE);
    n.getParam("TOPIC/PAN_CMD",         TOPIC_PAN_CMD);
    n.getParam("TOPIC/TILT_CMD",        TOPIC_TILT_CMD);
    n.getParam("GAIN/PAN/P",            GAIN_PAN_P);
    n.getParam("GAIN/PAN/I",            GAIN_PAN_I);
    n.getParam("GAIN/PAN/D",            GAIN_PAN_D);
    n.getParam("GAIN/TILT/P",           GAIN_TILT_P);
    n.getParam("GAIN/TILT/I",           GAIN_TILT_I);
    n.getParam("GAIN/TILT/D",           GAIN_TILT_D);
    // n.getParam("omomi_yokohaba",           OMOMI1);
    // n.getParam("omomi_kyori",           OMOMI2);
}
