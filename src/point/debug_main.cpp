#include<mynteye_pointcloud/point.h>

//このプログラムは受け取ったデータを外部に送信します。
int main(int argc,char **argv){
	ros::init(argc,argv,"mynteye_pointcloud_cd");

    pointClass pc;
    ros::spin();
    // pc.mainloop();

	return 0;
}