#pragma once

#include <ros/ros.h>
#include <time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

#include <opencv2/video/background_segm.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/PointCloud2.h>
#include<opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>


using namespace cv;
using namespace std;
const float RESO = 0.15f;
/**
 *  Resolution and time consuming
 *  Voxel / m               Time
 *  0.2         -->     0.000194183 ms
 *  0.1         -->     145.716 ms
 *  0.05        -->     Leaf size is too small for the input dataset.
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
    void Spin();
};