#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PLYSaver
{
public:
    PLYSaver(const std::string& save_path, bool binary)
        : save_path_(save_path), binary_(binary), saved_(false) {}

    void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        if (saved_) return;  // 只保存一次

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        ROS_INFO("收到点云，共 %zu 个点，正在保存到 %s", cloud.size(), save_path_.c_str());

        int ret;
        if (binary_)
            ret = pcl::io::savePLYFileBinary(save_path_, cloud);
        else
            ret = pcl::io::savePLYFileASCII(save_path_, cloud);

        if (ret == 0)
            ROS_INFO("点云保存成功: %s", save_path_.c_str());
        else
            ROS_ERROR("点云保存失败！");

        saved_ = true;
        ros::shutdown();  // 保存完退出
    }

private:
    std::string save_path_;
    bool binary_;
    bool saved_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "save_first_ply");
    ros::NodeHandle nh("~");

    std::string save_path;
    nh.param<std::string>("save_path", save_path, std::string("/home/zhywwyzh/workspace/VLA_Diff/unity/output_3dgs.ply"));
    bool binary;
    nh.param<bool>("binary", binary, true);

    PLYSaver saver(save_path, binary);
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(
        "/ply_points", 1, &PLYSaver::callback, &saver);

    ROS_INFO("等待 /ply_points 第一帧点云...");
    ros::spin();

    return 0;
}
