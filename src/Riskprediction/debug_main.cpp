#include<mynteye_pointcloud/Riskprediction.h>

//このプログラムは受け取ったデータを外部に送信します。
int main(int argc,char **argv){
	ros::init(argc,argv,"mynteye_pointcloud_rc");

    RiskClass rp;
    ros::spin();

	return 0;
}