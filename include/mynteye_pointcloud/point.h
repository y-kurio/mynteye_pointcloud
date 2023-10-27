#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <cmath>

//msg
#include <mynteye_pointcloud/ClassificationData.h>

//点群
#include <pcl_ros/segmentation/sac_segmentation.h>
#include <pcl_ros/segmentation/extract_clusters.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/kdtree/kdtree_flann.h>
//データ型
#include <mynteye_pointcloud/PointCloudData.h>



//クラスの定義
class pointClass{
    private:
        ros::NodeHandle pnh;
        //カメラデータ受診
		ros::NodeHandle nh;
		ros::Subscriber sub_point_, point_sub_;
        //sensor_msgs::PointCloud2 rawPC2,PC2;
        mynteye_pointcloud::ClassificationData closest_pt;
        double closest_distance_;
        //点群データ
        pcl::PointCloud<pcl::PointXYZ> pcloud;
        mynteye_pointcloud::ClassificationData Clstr;
        // pcl::PointCloud<pcl::PointXYZ> closest_point;
        mynteye_pointcloud::PointCloudData cloud;
        //カメラ変換データ送信
        ros::Publisher pubpcloud_, pubrawcloud_, pubkd_, closest_point_pub_;
        float X_,Y_,Z_;
        double EXTRACT_XMIN = -std::numeric_limits<double>::infinity(), 
        EXTRACT_XMAX = std::numeric_limits<double>::infinity(), 
        EXTRACT_YMIN = -std::numeric_limits<double>::infinity(), 
        EXTRACT_YMAX = std::numeric_limits<double>::infinity(),
        EXTRACT_ZMIN = -std::numeric_limits<double>::infinity(), 
        EXTRACT_ZMAX = std::numeric_limits<double>::infinity();
    public:
    //コンストラクタ：クラス定義に呼び出されるメソッド
        pointClass();
        //デストラクタ：クラスが消滅するときに呼びだされるメソッド
        ~pointClass();
        //メソッド：関数のようなもの:後でlaunchファイルからの読み込みメソッドを追加
        //in property.cpp
        //セット：内部パラメータの書き込み
        void setLaunchParam();//launchファイルから書き込み
        //in methods.cpp
        //--センサーデータ受信
        void pcloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
        void Cluster_callback(const mynteye_pointcloud::ClassificationData::ConstPtr& msg);
        void manage();
        // void Clustering();
        //点群の抜出
        void Extract();
        void minpt();
        //センサデータ送信
        void publishPointCloud();//データ送信
        //データクリア
        void clearMessages();
};