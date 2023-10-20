#include<mynteye_pointcloud/PointCloud.h>

void PointCloudClass::setLaunchParam(){
    
    ros::NodeHandle n("~");
    n.getParam("IS_MOVING",IS_MOVING);
    n.getParam("DISTANCE_TO_OBJECT",DISTANCE_TO_OBJECT);
    n.getParam("ANGLE_TO_OBJECT",ANGLE_TO_OBJECT);
    n.getParam("OBJECTSIZE_HEIGHT",OBJECTSIZE_HEIGHT);
    n.getParam("OBJECTSIZE_WIDTH",OBJECTSIZE_WIDTH);
    n.getParam("OBJECTSIZE_DEPTH",OBJECTSIZE_DEPTH);
    n.getParam("CAMERAPOS_HEIGHT",CAMERAPOS_HEIGHT);
    n.getParam("PIXEL_TO_XYZ",PIXEL_TO_XYZ);

    n.getParam("Extract/Xmin",EXTRACT_XMIN);
    n.getParam("Extract/Xmax",EXTRACT_XMAX);
    n.getParam("Extract/Ymin",EXTRACT_YMIN);
    n.getParam("Extract/Ymax",EXTRACT_YMAX);
}