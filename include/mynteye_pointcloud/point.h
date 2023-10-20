#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
//点群
#include <pcl_ros/segmentation/sac_segmentation.h>
#include <pcl_ros/segmentation/extract_clusters.h>
#include <pcl_ros/filters/voxel_grid.h>
//データ型
#include <mynteye_pointcloud/PointCloudData.h>



//クラスの定義
class pointClass{
    private:
        ros::NodeHandle pnh;
        //カメラデータ受診
		ros::NodeHandle nh;
		ros::Subscriber sub_point_;
        //sensor_msgs::PointCloud2 rawPC2,PC2;
        //点群データ
        pcl::PointCloud<pcl::PointXYZ> pcloud;
        mynteye_pointcloud::PointCloudData cloud;
        //カメラ変換データ送信
        ros::Publisher pubpcloud_, pubrawcloud_;
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
        void manage();
        void Clustering();
        //点群の抜出
        void Extract();
        //センサデータ送信
        void publishPointCloud();//データ送信
        //データクリア
        void clearMessages();
};