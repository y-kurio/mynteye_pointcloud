#include<mynteye_pointcloud/synchroImage.h>

void syncroImageClass::setLaunchParam(){
    
    ros::NodeHandle n("~");
    n.getParam("debugType",debugType);
}
