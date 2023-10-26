#include<mynteye_pointcloud/Riskprediction.h>

void syncroImageClass::setLaunchParam(){
    
    ros::NodeHandle n("~");
    n.getParam("debugType",debugType);
}
