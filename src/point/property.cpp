#include<mynteye_pointcloud/point.h>

void pointClass::setLaunchParam(){
    pnh.getParam("Xmin",EXTRACT_XMIN);
    pnh.getParam("Xmax",EXTRACT_XMAX);
    pnh.getParam("Ymin",EXTRACT_YMIN);
    pnh.getParam("Ymax",EXTRACT_YMAX);
    pnh.getParam("Zmin",EXTRACT_YMIN);
    pnh.getParam("Zmax",EXTRACT_YMAX);
}