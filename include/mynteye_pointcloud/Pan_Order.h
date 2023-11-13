#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

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
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <pcl_ros/point_cloud.h>

#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <cmath>

//msg
// #include <mynteye_pointcloud/ClassificationData.h>
#include <mynteye_pointcloud/pointData.h>
#include <mynteye_pointcloud/SetAngle.h>




//クラスの定義
class OrderClass{
    private:
        ros::NodeHandle pnh_;
		ros::NodeHandle nh_;
        ros::Subscriber sub_angle_;
        ros::Publisher pub_psition_;
        dynamixel_sdk_examples::SetPosition pubPanData_;
        double most_Cluster_theta_;
        mynteye_pointcloud::SetAngle camera_angle_;

        void __angle_callback(const mynteye_pointcloud::SetAngle& msg);
        void __setLaunchParam();
        void __riskobject();
        void __pantilt_order();
        void __publish();//データ送信

    public:
        OrderClass();
        ~OrderClass();
        void mainloop();
        
};