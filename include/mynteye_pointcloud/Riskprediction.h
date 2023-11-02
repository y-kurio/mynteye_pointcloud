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
#include <mynteye_pointcloud/ClassificationData.h>
#include <mynteye_pointcloud/pointData.h>




//クラスの定義
class RiskClass{
    private:
        ros::NodeHandle pnh;
		ros::NodeHandle nh;
		ros::Subscriber sub_close_point_;
        ros::Publisher Risk_pub_, pub_pan_, pub_tilt_;
        mynteye_pointcloud::pointData Cluster_Minpts, Cluster_risk;
        dynamixel_sdk_examples::SetPosition pubPanData;
    public:
    //コンストラクタ：クラス定義に呼び出されるメソッド
        RiskClass();
        //デストラクタ：クラスが消滅するときに呼びだされるメソッド
        ~RiskClass();
        //メソッド：関数のようなもの:後でlaunchファイルからの読み込みメソッドを追加
        //in property.cpp
        //セット：内部パラメータの書き込み
        void setLaunchParam();//launchファイルから書き込み
        //in methods.cpp
        //--センサーデータ受信
        void close_point_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
        void manage();
        void riskobject();
        void pantilt_order();
        void publish();//データ送信
};