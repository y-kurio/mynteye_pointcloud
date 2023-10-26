#include <mynteye_pointcloud/synchroImage.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"mynteye_pointcloud_si");
	
    syncroImageClass sic; //syncro class

    ros::spin();

	return 0;
}