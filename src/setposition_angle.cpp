#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <cmath>
// pantilt
#include <dynamixel_sdk_examples/GetPosition.h>
#include <dynamixel_sdk_examples/SetPosition.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
//msg
#include <mynteye_pointcloud/ClassificationData.h>
//データ型
#include <ros/callback_queue.h>
#include <mynteye_pointcloud/pointData.h>
#include <mynteye_pointcloud/SetAngle.h>

void __angle_callback(const mynteye_pointcloud::SetAngle& msg)
{
    mynteye_pointcloud::SetAngle camera_angle_ = msg;
} 

int main(int argc,char **argv){
	ros::init(argc,argv,"setposition_angle");
    ros::NodeHandle pnh_;
    ros::NodeHandle nh_;
    ros::Subscriber sub_angle_;
    ros::Publisher pub_psition_;
    sub_angle_ =nh_.subscribe("set_angle",1,&RiskClass::__angle_callback,this);
    pub_psition_=nh_.advertise<dynamixel_sdk_examples::SetPosition>("set_position",1);
    dynamixel_sdk_examples::SetPosition pubPanData_;
    double most_Cluster_theta_;
    mynteye_pointcloud::SetAngle camera_angle_;
    pnh_.getParam("IS_MOVING",HZ);
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
    most_Cluster_theta_ = (camera_angle_.angle)/M_PI*180;
    pubPanData_.id = camera_angle_.id;
    pubPanData_.position = int((most_Cluster_theta_*11.6) + 2048);
    pub_psition_.publish(pubPanData_);
        ros::spinOnce();
        loop_rate.sleep();
    }

	return 0;
}