#include "pcl_core.h"

PclTestCore::PclTestCore(ros::NodeHandle &nh){
    sub_point_cloud_ = nh.subscribe("/points_raw",10, &PclTestCore::point_cb, this);

    pub_filtered_points_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 10);

    ros::spin();
}

PclTestCore::~PclTestCore(){}

void PclTestCore::Spin(){

}


Mat gray2pseudocolor(const Mat& scaledGray)
{
    Mat outputPseudocolor(scaledGray.size(), CV_8UC3);
    unsigned char grayValue;
    for (int y = 0; y < scaledGray.rows; y++)
        for (int x = 0; x < scaledGray.cols; x++)
        {
            grayValue = scaledGray.at<uchar>(y, x);
            Vec3b& pixel = outputPseudocolor.at<Vec3b>(y, x);
            pixel[0] = abs(255 - grayValue);
            pixel[1] = abs(127 - grayValue);
            pixel[2] = abs(0 - grayValue);
        }

    return outputPseudocolor;
}
Mat gray2rainbow(const Mat& scaledGray)
{
    Mat outputRainbow(scaledGray.size(), CV_8UC3);
    unsigned char grayValue;
    for (int y = 0; y < scaledGray.rows; y++)
        for (int x = 0; x < scaledGray.cols; x++)
        {
            grayValue = scaledGray.at<uchar>(y, x);
            Vec3b& pixel = outputRainbow.at<Vec3b>(y, x);
            if(grayValue == 0)
            {
                pixel[0] = 0;
                pixel[1] = 0;
                pixel[2] = 0;
            }
            else if (grayValue <= 51)
            {
                pixel[0] = 255;
                pixel[1] = grayValue * 5;
                pixel[2] = 0;
            }
            else if (grayValue <= 102)
            {
                grayValue -= 51;
                pixel[0] = 255 - grayValue * 5;
                pixel[1] = 255;
                pixel[2] = 0;
            }
            else if (grayValue <= 153)
            {
                grayValue -= 102;
                pixel[0] = 0;
                pixel[1] = 255;
                pixel[2] = grayValue * 5;
            }
            else if (grayValue <= 204)
            {
                grayValue -= 153;
                pixel[0] = 0;
                pixel[1] = 255 - static_cast<unsigned char>(grayValue * 128.0 / 51 + 0.5);
                pixel[2] = 255;
            }
            else if (grayValue <= 255)
            {
                grayValue -= 204;
                pixel[0] = 0;
                pixel[1] = 127 - static_cast<unsigned char>(grayValue * 127.0 / 51 + 0.5);
                pixel[2] = 255;
            }
        }

    return outputRainbow;
}
// 创建全局常亮
int map_width,map_height,offset_x,offset_y;
// 创建缓存矩阵
Mat last,diff;
void point_cloud_2_birdseye(pcl::PointCloud<pcl::PointXYZI>::Ptr & pc, cv::Mat & out2){
    clock_t start,end;
    start = clock();
    //需要测试运行时间的程序段
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width  = pc->points.size();
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for(size_t i = 0; i < pc->points.size(); i++){
        cloud->points[i].x = pc->points[i].x;
        cloud->points[i].y = pc->points[i].y;
        cloud->points[i].z = pc->points[i].z;
    }
    pcl::PointXYZ min;//用于存放三个轴的最小值
    pcl::PointXYZ max;//用于存放三个轴的最大值
    pcl::getMinMax3D(*cloud,min,max);
    static int flag1 = 0;

    if(flag1 == 0){
        // 读取图片合理尺寸
        map_width  = (int)((max.x - min.x)/RESO)+50;
        map_height = (int)((max.y - min.y)/RESO)+50;
        // 创建尺寸变量
        Size dsize = Size(map_height,map_width);
        // 更新缓存图片尺寸
        last.create(dsize,CV_8UC1);
        diff.create(dsize,CV_8UC1);
        // 更新点云平面到像素平面的偏置
        offset_x = min.x;
        offset_y = min.y;
        //保证只执行一次
        flag1=1;
    }

    int z_scale = (int)((max.z-min.z));
    // 创建输出矩阵并清零.
    Mat out(map_width,map_height,CV_8UC1,Scalar(0));

    // 遍历点云并赋值到矩阵
    for(size_t i = 0; i < pc->points.size(); i++){
        size_t Y =(int)((cloud->points[i].y - offset_y)/(RESO));
        size_t X =(int)((cloud->points[i].x - offset_x)/(RESO));
        uchar Z = (uchar)((cloud->points[i].z - min.z)/(z_scale) *255);
        uchar * inData = out.ptr<uchar>(X);
        inData[Y] = Z;
    }
    // 进行矩阵减法
    absdiff(out,last,diff);
    // 更新矩阵
    last = out;
    // 灰度图像彩色化
    Mat color(map_width,map_height,CV_8UC3,Scalar(0));
    color = gray2rainbow(diff);

    cv::imshow("bird", color);
    waitKey(1);
    end = clock();
    // 计算运行时间
    cout<<"Time consuming: "<<(double)(end-start)/CLOCKS_PER_SEC/1000<<" ms"<<endl;
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
    vg.setLeafSize(RESO, RESO, RESO);
    vg.filter(*filtered_pc_ptr);
    cv::Mat M;
    point_cloud_2_birdseye(current_pc_ptr,M);
    //创建ROS类型
    sensor_msgs::PointCloud2 pub_pc;
    pcl::toROSMsg(*filtered_pc_ptr, pub_pc);
    pub_pc.header = in_cloud_ptr->header;

    pub_filtered_points_.publish(pub_pc);
}


