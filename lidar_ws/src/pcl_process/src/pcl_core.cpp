#include "pcl_core.h"

PclTools pcl_tools;

class StatisticalOutlierRemoval;

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
    clock_t start,end;


    pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr height_filted(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr knn     (new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud   (new pcl::PointCloud<pcl::PointXYZ>);
    // 从ROS中的PointCloud2数据类型转换为PCL的数据类型
    pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);
    for(size_t i=0;i<current_pc_ptr->size();i++){
        if(current_pc_ptr->points[i].z>Z_min&&current_pc_ptr->points[i].z<Z_max)
            height_filted->push_back(current_pc_ptr->points[i]);

    }
    // 创建VoxelGrid对象
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    // 对图像进行分割
    vg.setInputCloud(height_filted);
    vg.setLeafSize(RESO, RESO, RESO);
    vg.filter(*filtered_pc_ptr);
    start = clock();

//    for(size_t i = 0; i < filtered_pc_ptr->points.size(); i++){
//        cloud->points[i].x = filtered_pc_ptr->points[i].x;
//        cloud->points[i].y = filtered_pc_ptr->points[i].y;
//        cloud->points[i].z = filtered_pc_ptr->points[i].z;
//    }
//    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
//    sor.setInputCloud(filtered_pc_ptr);
//    sor.setMeanK(50); // 设置 m 为在进行统计时考虑查询点邻近点数
//    sor.setStddevMulThresh(0.2); // 设置距离阈值，其公式是 mean + global stddev_mult * global stddev，即mean+1.0*stddev
//    sor.filter(*knn); // 执行去噪计算并保存点到 cloud_filtered
/*
 * 直通滤波器
 * */
// Create the filtering object
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud (filtered_pc_ptr);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
//pass.setFilterLimitsNegative (true);
    pass.filter (*filtered_pc_ptr);
/*
 * 半径滤波器
 * */
    // build the filter
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
    outrem.setInputCloud(filtered_pc_ptr);
    outrem.setRadiusSearch(0.5);
    outrem.setMinNeighborsInRadius (5);
    // apply filter
    outrem.filter (*filtered_pc_ptr);


    static int flag = 0;
    if(flag ==0){
        pcl_tools.Init2D(height_filted);
        flag=1;
    }

    cv::Mat M;
    pcl_tools.pc = height_filted;
    pcl_tools.point_cloud_2_birdseye();
    end = clock();
    //创建ROS类型
    sensor_msgs::PointCloud2 pub_pc;
    pcl::toROSMsg(*(pcl_tools.oout), pub_pc);
    pub_pc.header = in_cloud_ptr->header;
    pub_filtered_points_.publish(pub_pc);

//    cout<<"FPS: "<<(double)((double)(CLOCKS_PER_SEC)/(end-start))<<endl;
}


