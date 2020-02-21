#pragma once

#include <ros/ros.h>
#include <time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/video/background_segm.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/time.h>
#include <pcl/common/angles.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>


#include <sensor_msgs/PointCloud2.h>

#include <iostream>


using namespace cv;
using namespace std;
const float RESO = 0.15f,RESO_2D = 0.2f;
const float Z_max = 0.2f,Z_min = -1.9;
/**
 *  Resolution and time consuming
 *  Voxel / m               Time
 *  0.2         -->     0.000194183 ms
 *  0.1         -->     145.716 ms
 *  0.05        -->     Leaf size is too small for the input dataset.
 *
 *  Do the Voxel filter takes no time
 * */
class PclTestCore
{

private:
    ros::Subscriber sub_point_cloud_;

    ros::Publisher pub_filtered_points_;

    void point_cb(const sensor_msgs::PointCloud2ConstPtr& in_cloud);

public:
    PclTestCore(ros::NodeHandle &nh);
    ~PclTestCore();
//    pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_ptr;
    void Spin();
};

class PclTools{
private:

public:
    Mat I1x1,I2x2,I3x3,I5x5;
    pcl::PointXYZ point_min;//用于存放三个轴的最小值
    pcl::PointXYZ point_max;//用于存放三个轴的最大值
    Mat masks,AverageImage;
    int map_width,map_height,offset_x,offset_y,z_scale ;
    Size dsize;
    // 创建全局常量
    int counter = 0;
    Ptr<BackgroundSubtractorMOG2> bg;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc;
    pcl::PointCloud<pcl::PointXYZI>::Ptr oout;
    PclTools();
    ~PclTools();
    Mat gray2rainbow(const Mat& scaledGray);
    void Init2D(pcl::PointCloud<pcl::PointXYZI>::Ptr & pc_init);
    void point_cloud_2_birdseye();
    void find_moving_obj(Mat & src);

};