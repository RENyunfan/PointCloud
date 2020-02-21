//
// Created by kevin on 2/19/20.
//
#include <pcl_core.h>
#include "pcl_core.h"

PclTools::PclTools(){
    I1x1 = Mat::ones(1, 1, CV_8UC1);
    I2x2 = Mat::ones(2, 2, CV_8UC1);
    I3x3 = Mat::ones(3, 3, CV_8UC1);
    I5x5 = Mat::ones(5, 5, CV_8UC1);
}
PclTools::~PclTools(){}

void PclTools::Init2D(pcl::PointCloud<pcl::PointXYZI>::Ptr & pc_init){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width  = pc_init->points.size();
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);
    for(size_t i = 0; i < pc_init->points.size(); i++){
        cloud->points[i].x = pc_init->points[i].x;
        cloud->points[i].y = pc_init->points[i].y;
        cloud->points[i].z = pc_init->points[i].z;
    }
    // 读取图片合理尺寸
    pcl::getMinMax3D(*cloud,point_min,point_max);
    map_width  = (int)((point_max.x - point_min.x)/RESO_2D)+20;
    map_height = (int)((point_max.y - point_min.y)/RESO_2D)+20;
    // 创建尺寸变量
    dsize = Size(map_height,map_width);
    // 更新缓存图片尺寸
    AverageImage.create(dsize,CV_32FC1);
    // 更新点云平面到像素平面的偏置
    offset_x = point_min.x;
    offset_y = point_min.y;
    z_scale = (int)((point_max.z-point_min.z))+10;
    // 高斯混合模型
    bg = createBackgroundSubtractorMOG2(10000,100, true);
}

Mat PclTools::gray2rainbow(const Mat& scaledGray)
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

void PclTools::point_cloud_2_birdseye(){

    // 创建输出矩阵并清零.
    Mat out(map_width,map_height,CV_8UC1,Scalar(0));
    clock_t a,b;

    // 遍历点云并赋值到矩阵
    for(size_t i = 0; i < pc->points.size(); i++) {
        size_t Y = (int) ((pc->points[i].y - offset_y) / (RESO_2D));
        size_t X = (int) ((pc->points[i].x - offset_x) / (RESO_2D));
//        cout << pc->points[i].z << endl;
        uchar Z = (uchar) ((pc->points[i].z - Z_min) / (Z_max-Z_min) * 255);
        uchar *inData = out.ptr<uchar>(X);
        inData[Y] = Z;

    }
    b = clock();
    bg->setNMixtures(7);
    bg->apply(out,masks,-1);
    a = clock();
//    // 灰度图像彩色化
    Mat color(map_width,map_height,CV_8UC3,Scalar(0));
    color = gray2rainbow(out);
    Mat dilated,result,eroded;
    morphologyEx(masks, eroded, cv::MORPH_CLOSE, I2x2);
////    erode(masks,result,I2x2);
////    GaussianBlur(masks,result,Size(3, 3 ),0 ,0);
//    dilate(masks, dilated, getStructuringElement(MORPH_RECT, Size(5, 5)));
//    erode(dilated, eroded, getStructuringElement(MORPH_RECT, Size(5, 5)));
    medianBlur(eroded,result,3);
    imshow("bird", color);
//    imshow("res", dilated);
    find_moving_obj(result);
    imshow("msk", result);
//    imshow("sum", AverageImage);
//    backgroudnSubtract(out);
    waitKey(1);
    cout<<"process: "<<(double)((double)(CLOCKS_PER_SEC)/(a-b))<<endl;

}


int thresh = 100;
int max_thresh = 255;
RNG rng(12345);
void PclTools::find_moving_obj(Mat & src_gray){

        Mat threshold_output;
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;

        /// 使用Threshold检测边缘
        threshold(src_gray, threshold_output, thresh, 255, THRESH_BINARY);
        /// 找到轮廓
        findContours(threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

        /// 多边形逼近轮廓 + 获取矩形和圆形边界框
       // vector<vector<Point> > contours_poly(contours.size());
        vector<Rect> boundRect(contours.size());
        vector<Point2f>center(contours.size());
        vector<float>radius(contours.size());

        //for (int i = 0; i < contours.size(); i++)
        //{
        //	approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
        //	boundRect[i] = boundingRect(Mat(contours_poly[i]));
        //	minEnclosingCircle(contours_poly[i], center[i], radius[i]);
        //}
        int index = 0;
        for (int i = 0; i < contours.size(); i++)
        {

                boundRect[i] = boundingRect(Mat(contours[i]));
           // approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);

           // minEnclosingCircle(contours[i], center[i], radius[i]);
        }

        Mat drawing = Mat::zeros(threshold_output.size(), CV_8UC3);
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_(new pcl::PointCloud<pcl::PointXYZI>);
        for (int i = 0; i< contours.size(); i++)
        {
            Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
          //  drawContours(drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point());
            rectangle(src_gray, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0);
            //circle(drawing, center[i], (int)radius[i], color, 2, 8, 0);
            for(size_t j=0; j< pc->size();j++)
            {

                if((pc->points[j].x>((double)(boundRect[i].y-3)*RESO_2D)+offset_x)
                    &&(pc->points[j].x<((double)(boundRect[i].y+3+boundRect[i].width)*RESO_2D+offset_x))
                      &&(pc->points[j].y<((double)(boundRect[i].x+3+boundRect[i].height)*RESO_2D+offset_y))
                        &&(pc->points[j].y>((double)(boundRect[i].x-3)*RESO_2D)+offset_y) )
                {
                    out_->push_back(pc->points[j]);
                }
            }
        }
        oout = out_;

        /// 显示在一个窗口
        //namedWindow("Contours", CV_WINDOW_AUTOSIZE);
       /// imshow("Contours", drawing);


}

