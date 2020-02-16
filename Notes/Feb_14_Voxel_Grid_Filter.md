# Getting started with PCL

> Yunfan REN
>
> 02/14/2020
>
> renyunfan@berkeley.edu

# 1 在ROS项目中添加PCL库

首先在[PCL官网](http://www.pointclouds.org/downloads/linux.html)，按照指导顺序安装PCL库。本文档假设读者已自行安装好ROS Kinetic以及PCL库。随后我们创建一个工作空间。

```bash
mkdir -p lidar_ws/src
cd lidar_ws
catkin_make
cd src
catkin_create_pkg pcl_test roscpp sensor_msgs pcl_ros
```

以上步骤创建了一个工作空间和一个rospackage。

随后配置CMakeList文件，创建源文件。

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(pcl_test)
 
add_compile_options(-std=c++11)
 
 
find_package(catkin REQUIRED COMPONENTS
pcl_ros
roscpp
sensor_msgs
)
 
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
)
 
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
 
 
add_executable(${PROJECT_NAME}_node src/pcl_test_node.cpp src/pcl_test_core.cpp)
 
 
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
```

在这里我们主要引入了三个库文件，包括一个ROS的文件，一个cpp的库，以及一个用于
ROS消息的sensor_msgs库。

同样的，在CMakeList中，我们通过

* `find_package`查找这三个包的路径；

* 将三个包添加到 `CATKIN_DEPENDS`, 

* 将PCL库的路径链接，通过`link_directories(${PCL_LIBRARY_DIRS})`最后的`target_link_libraries`中添加`${PCL_LIBRARIES}`。

# 2 编写节点进行Voxel Grid Filter 

体素降采样

我们采用ROS文件和处理文件分离的思路，首先写一个单独的文件作为ROS句柄的创建和ROS消息的收发。

```cpp
#include "pcl_test_core.h"
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_test");
 
    ros::NodeHandle nh;
 
    PclTestCore core(nh);
    return 0;
}
```

随后我们编写头文件，申明帮助我们处理点云的类

```cpp
#pragma once
 
#include <ros/ros.h>
 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
 
#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/PointCloud2.h>
 
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
```

之后编写我们最重要的处理函数源文件，它将通过头文件和ROS消息文件连接起来，实现点云数据的订阅和处理，以及处理后数据的再发布。

```cpp
#include "pcl_test_core.h"
 
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
    vg.setLeafSize(0.2f, 0.2f, 0.2f);
    vg.filter(*filtered_pc_ptr);
    //创建ROS类型
    sensor_msgs::PointCloud2 pub_pc;
    pcl::toROSMsg(*filtered_pc_ptr, pub_pc);
    pub_pc.header = in_cloud_ptr->header;
 
    pub_filtered_points_.publish(pub_pc);
}
```

这个节点订阅了来自*\points_raw*话题的点云数据，使用PCL内置的Voxel Grid Filter 对原始点云数据进行采样，再讲采样的结果发布到*\filtered_points*话题上。为了使用Voxel Grid Filter对原始点云进行降采样，只需定义`pcl::VocelGrid`并且指定输入点云和leaf size，在本例中，我们使用leaf size为 0.2。Voxel Grid Filter将输入点云使用0.2m*0.2m*0.2m的立方体进行分割，使用小立方体的 **形心（centroid）** 来表示这个立方体的所有点，保留这些点作为降采样的输出。

> Voxel 体积元素, 也称为体素, 是三维空间表示的一种方法 .
>
> 体素是体积元素（Volume Pixel）的简称，包含体素的立体可以通过[立体渲染](https://baike.baidu.com/item/立体渲染/535400)或者提取给定[阈值](https://baike.baidu.com/item/阈值/7442398)轮廓的多边形[等值面](https://baike.baidu.com/item/等值面/10488111)表现出来。一如其名，是数字数据于[三维空间](https://baike.baidu.com/item/三维空间/3180500)分割上的最小单位，体素用于三维成像、科学数据与[医学影像](https://baike.baidu.com/item/医学影像/4954291)等领域。概念上类似二维空间的最小单位——像素，像素用在二维计算机图像的影像数据上。有些真正的三维显示器运用体素来描述它们的分辨率，举例来说：可以显示512×512×512体素的显示器。

# 3 点云地面过滤-Ray Ground Filter

> RGF算法的优点是计算量比较小,缺点是只能计算一个平面的地面,如果出现坑洼,街边的凸起道路,那就完全无法判断.如果由地面突然凸起也是没办法判断的.

过滤地面是激光雷达感知中一步基础的预处理操作，因为我们环境感知通常只对路面上的障碍物感兴趣，且地面的点对于障碍物聚类容易产生影响，所以在做Lidar Obstacle Detection之前通常将地面点和非地面点进行分离。在此文中我们介绍一种被称为Ray Ground Filter的路面过滤方法，并且在ROS中实践。

### 裁剪点云

第一步,我们先进行高度分割,删除掉过高的区域. 例如我们先删除到高于1.28米的部分

```cpp
void PclTestCore::clip_above(double clip_height, const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                             const pcl::PointCloud<pcl::PointXYZI>::Ptr out)
{
    pcl::ExtractIndices<pcl::PointXYZI> cliper;
 
    cliper.setInputCloud(in);
    pcl::PointIndices indices;
#pragma omp for
    for (size_t i = 0; i < in->points.size(); i++)
    {
        if (in->points[i].z > clip_height)
        {
            indices.indices.push_back(i);
        }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true); //ture to remove the indices
    cliper.filter(*out);
}
```

## 角度微分和地面非地面判断

将距离表示为极坐标, 我们使用一下数据结构来代替`pcl::PointCloudXYZI`

```cpp
  struct PointXYZIRTColor
  {
    pcl::PointXYZI point;
 
    float radius; //cylindric coords on XY Plane
    float theta;  //angle deg on XY plane
 
    size_t radial_div;     //index of the radial divsion to which this point belongs to
    size_t concentric_div; //index of the concentric division to which this points belongs to
 
    size_t original_index; //index of this point in the source pointcloud
  };
  typedef std::vector<PointXYZIRTColor> PointCloudXYZIRTColor;
```

其中用`radius`表示点到lidar的距离(半径)
$$
radius = x^2+y^2
$$
`theta`是点相对于车头方向的夹角.
$$
\theta = \arctan \frac{y}{x}\cdot \frac{180}{\pi}
$$
而另外两个变量我们用`radial_div`和`concentric_div`分别描述角度微分和距离微分。

将所有线的距离进行排序

```cpp
    //将同一根射线上的点按照半径（距离）排序
#pragma omp for
    for (size_t i = 0; i < radial_dividers_num_; i++)
    {
        std::sort(out_radial_ordered_clouds[i].begin(), out_radial_ordered_clouds[i].end(),[](const PointXYZIRTColor &a, const PointXYZIRTColor &b) { return a.radius < b.radius; });
    }
```

随后计算两点之间的坡度,从而判断是否为地面点

```cpp
void PclTestCore::classify_pc(std::vector<PointCloudXYZIRTColor> &in_radial_ordered_clouds,
                              pcl::PointIndices &out_ground_indices,
                              pcl::PointIndices &out_no_ground_indices)
{
    out_ground_indices.indices.clear();
    out_no_ground_indices.indices.clear();
#pragma omp for
    for (size_t i = 0; i < in_radial_ordered_clouds.size(); i++) //sweep through each radial division 遍历每一根射线
    {
        float prev_radius = 0.f;
        float prev_height = -SENSOR_HEIGHT;
        bool prev_ground = false;
        bool current_ground = false;
        for (size_t j = 0; j < in_radial_ordered_clouds[i].size(); j++) //loop through each point in the radial div
        {
            float points_distance = in_radial_ordered_clouds[i][j].radius - prev_radius;
            float height_threshold = tan(DEG2RAD(local_max_slope_)) * points_distance;
            float current_height = in_radial_ordered_clouds[i][j].point.z;
            float general_height_threshold = tan(DEG2RAD(general_max_slope_)) * in_radial_ordered_clouds[i][j].radius;
 
            //for points which are very close causing the height threshold to be tiny, set a minimum value
            if (points_distance > concentric_divider_distance_ && height_threshold < min_height_threshold_)
            {
                height_threshold = min_height_threshold_;
            }
 
            //check current point height against the LOCAL threshold (previous point)
            if (current_height <= (prev_height + height_threshold) && current_height >= (prev_height - height_threshold))
            {
                //Check again using general geometry (radius from origin) if previous points wasn't ground
                if (!prev_ground)
                {
                    if (current_height <= (-SENSOR_HEIGHT + general_height_threshold) && current_height >= (-SENSOR_HEIGHT - general_height_threshold))
                    {
                        current_ground = true;
                    }
                    else
                    {
                        current_ground = false;
                    }
                }
                else
                {
                    current_ground = true;
                }
            }
            else
            {
                //check if previous point is too far from previous one, if so classify again
                if (points_distance > reclass_distance_threshold_ &&
                    (current_height <= (-SENSOR_HEIGHT + height_threshold) && current_height >= (-SENSOR_HEIGHT - height_threshold)))
                {
                    current_ground = true;
                }
                else
                {
                    current_ground = false;
                }
            }
 
            if (current_ground)
            {
                out_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
                prev_ground = true;
            }
            else
            {
                out_no_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
                prev_ground = false;
            }
 
            prev_radius = in_radial_ordered_clouds[i][j].radius;
            prev_height = in_radial_ordered_clouds[i][j].point.z;
        }
    }
}
```

这里有两个重要参数，一个是`local_max_slope_`，是我们设定的同条射线上邻近两点的坡度阈值，一个是`general_max_slope_`，表示整个地面的坡度阈值，这两个坡度阈值的单位为度（degree），我们通过这两个坡度阈值以及当前点的半径（到lidar的水平距离）求得高度阈值，通过判断当前点的高度（即点的z值）是否在地面加减高度阈值范围内来判断当前点是为地面。

在地面判断条件中，`current_height <= (-SENSOR_HEIGHT + general_height_threshold) && current_height >= (-SENSOR_HEIGHT - general_height_threshold)` 中SENSOR_HEIGHT表示lidar挂载的高度，-SNESOR_HEIGHT即表示水平地面。

# 4 PCL中的一些操作

## 4.1 利用ExtractIndices按点云索引提取点云

点云操作过程中经常会需要提取点云子集，包括一些点云滤波算法也会经常得到点云的索引，然后根据这些点云索引来提取点云子集，下面代码示例了如何利用索引向量来构建点云索引并提取点云子集。

```cpp
/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2018年10月29日
* Copyright    :
* Descriptoin  :
* References   :
======================================================================*/

#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
using std::cout; using std::endl;
int main()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width = 5;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }
  std::cerr << "Cloud before extract: " << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " "
    << cloud->points[i].y << " "
    << cloud->points[i].z << std::endl;

  std::vector<int> index = {1,3,4};//提取1,3,4位置处点云
  boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(index);
  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  // Extract the inliers
  extract.setInputCloud (cloud);
  extract.setIndices (index_ptr);
  extract.setNegative (false);//如果设为true,可以提取指定index之外的点云
  extract.filter (*cloud_p);

  cout<<"----------------"<<endl;
  for (size_t i = 0; i < cloud_p->points.size (); ++i)
      std::cerr << "    " << cloud_p->points[i].x << " "
      << cloud_p->points[i].y << " "
      << cloud_p->points[i].z << std::endl;
}
```

ExtractIndices类可以提供多种功能的点云子集提取，如果只是提取指定索引的点云组成一个新的点云，还可以用`pcl::copyPointCloud`函数：

```cpp
std::vector<int > indexs = { 1, 2, 5 };//声明索引值
pcl::copyPointCloud(*cloud, indexs, *cloud_p);//将对应索引的点存储
```

## 4.2 从点云中提取一个子集

首先创建一个VoxelGrid滤波器对数据进行下采样,进行下采样科技加速处理过程,减少分割循环.

```cpp
pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;	//体素栅格下的原始对象
sor.setInputCloud(cloud_blob);					//设置下采样原始点云数据
sor.setLeafSize(0.01f,0.01f,0.01f);				//设置体素大小
sor.filter(*cloud_filtered_blob);				//执行采样保存数据
```

随后我们将处理参数化分割, 具体原因我们留到后续分割的部分再详细学习.

```cpp
pcl::ModelCoefficients::Ptrcoefficients(new pcl::ModelCoefficients());
pcl::PointIndices::Ptrinliers(new pcl::ModelCoefficients());
pcl::SACSegmentation<pcl::PointXYZ> seg;  //创建分割对象
seg.setOptimizeCoefficients(true);        //设置对估计的模型参数进行优化处理
seg.setModelType(pcl::SACMODEL_PLANE);    //设置分割模型类别
seg.setMethodType(pcl::SAC_RANSAC);       //设置用哪个随机参数估计方法
seg.setMaxIterations(1000);                //设置最大迭代次数
seg.setDistanceThreshold(0.01);            //设置判断是否为模型内点的距离阈值
```

设置extraction filter的实际参数

```cpp
pcl::ExtractIndices<pcl::PointXYZ> extract; //创建点云提取对象
extract.setInputCloud(cloud_filtered);      //设置输入点云
extract.setIndices(inliers);                 //设置分割后的内点为需要提取的点集
extract.setNegative(false);                  //设置提取内点而非外点
extract.filter(*cloud_p);                    //提取输出存储到cloud_p
```

为了处理点云中包含多个模型，我们在一个循环中执行该过程，并在每次模型被提取后，我们保存剩余的点，进行迭代。模型内点通过分割过程获取，如下

```cpp
seg.setInputCloud(cloud_filtered);
seg.segment(*inliers,*coefficients);
```

