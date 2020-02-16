#include "pcl_core.h"

PclTestCore::PclTestCore(ros::NodeHandle &nh){
    sub_point_cloud_ = nh.subscribe("/points_raw",10, &PclTestCore::point_cb, this);

    pub_filtered_points_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 10);

    ros::spin();
}

PclTestCore::~PclTestCore(){}

void PclTestCore::Spin(){

}
/*
定义回调函数
并且使用Voxel Grid 对原始数据进行降采样

*/
void PclTestCore::point_cb(const sensor_msgs::PointCloud2ConstPtr & in_cloud_ptr){
    // 定义两个指针,其中PointCloud是一个模板类,而PointXYZI是一种数据结构
    // 定义了四个参数,包括三维空间点的坐标(XYZ)和一个反射强度
    // 反射强度一定程度上可以反应物体的颜色
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    // 从ROS中的PointCloud2数据类型转换为PCL的数据类型
    pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);
    // 创建VoxelGrid对象
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    // 对图像进行分割
    vg.setInputCloud(current_pc_ptr);
    vg.setLeafSize(1.0f, 1.0f, 1.0f);
    vg.filter(*filtered_pc_ptr);
    //创建ROS类型
    sensor_msgs::PointCloud2 pub_pc;
    pcl::toROSMsg(*filtered_pc_ptr, pub_pc);
    pub_pc.header = in_cloud_ptr->header;

    pub_filtered_points_.publish(pub_pc);
}