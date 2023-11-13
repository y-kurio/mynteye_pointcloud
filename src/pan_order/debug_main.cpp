#include<mynteye_pointcloud/Pan_Order.h>

//このプログラムは受け取ったデータを外部に送信します。
int main(int argc,char **argv){
	ros::init(argc,argv,"mynteye_pointcloud_po");

    OrderClass po;
    po.mainloop();

	return 0;
}