

# Gaussian Mixed Model

![](/home/kevin/Pictures/Gaussian.png)

# 1 基本原理

虽然是背景，但是灰度并不是一直保持不变的，灰度是在一个范围内变化的。一个背景像素随着时间变化呈现一定的随机性，但是一段时间内，如果做一个统计的话，其分布在某一个均值，一个方差范围内，绝大多数像素应该落在3σ范围内。图像中每一个像素灰度值随着时间变化的规律符合高斯分布。在实际中，我们发现灰度值的变化往往不能用一个高斯模型刻画，哪能不能用多个高斯模型去刻画呢？这是可以的，任何一种分布函数都可以看作是多个高斯分布的线性加权组合。
![在这里插入图片描述](https://img-blog.csdnimg.cn/20190320114547670.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2hvdGJveWJveQ==,size_16,color_FFFFFF,t_70)

回忆概率论中知识，正态分布可以看作满足$3\sigma$分布。如果当前点$I(x,y,z) - \mu>3\sigma$， 那么我们认为他是前景，否则是背景。
![在这里插入图片描述](https://img-blog.csdnimg.cn/2019032011470424.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2hvdGJveWJveQ==,size_16,color_FFFFFF,t_70)

像素灰度的概率密度函数
$$
p (I) = \sum_{q=1}^Qw_qN(I;\mu_q,\sigma_q^2)
\\
G(I;\mu_q,\sigma_q)  = \frac{1}{\sqrt{2\pi\sigma_q}}\exp(-\frac{(1-\mu)^2}{2\sigma^2_q})
$$

* $p$: 概率，多个高斯分布的甲加权组合。
* $I$： 亮度，灰度值。
* $w$：权重。
* $q$： 下标，代表是$q$个高斯分布。
* $\mu$：均值
* $\sigma$：方差

# 2 建模步骤

* 模型初始化，将采到的第一帧图像的每个像素的灰度值作为均值，再赋以较大的方差(比如10)，初值Q=1，w=1；假如图像为30*30，则需要建立900个混合高斯模型

* 模型学习 将当前帧的对应点像素的灰度值与已有的Q个高斯模型作比较，若满足|xk-u(q,k) |<2.5σ(q,k),则按下面迭代计算方式调整第q个高斯模型的参数和权重；否则转入（3）

* 增加/替换高斯分量，若不满足条件，且q<Q，则增加一个新分量；若q=Q，则替换（假设满足q=Q,需要把权值最小的那一个删掉，重新加一个）

* 判断背景

  ![在这里插入图片描述](https://img-blog.csdnimg.cn/20190320115327715.png)

* 判断前景

**计算原理**

![在这里插入图片描述](https://img-blog.csdnimg.cn/2019032011541959.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2hvdGJveWJveQ==,size_16,color_FFFFFF,t_70)

α和ρ对应的是学习速率
α越大，表示学习当前的东西越快，对以前的东西遗忘的越快，
理解：当下一帧的图像过来，对应的像素值去跟已有的所有高斯分量去比较，仅当像素值匹配第q类时二值化函数的值为1，反之为0，所有高斯分量的权值同时更新，其他两个参数同理。

# 3 代码实现

```cpp
// 创建类指针
Ptr<BackgroundSubtractorMOG2> bg;
// 创建类模型
bg = createBackgroundSubtractorMOG2();
// 设置模型参数
bg->setNMixtures(3);
// 模型匹配
bg->apply(out,masks);

```

## 参数分析

```cpp
Ptr<BackgroundSubtractorMOG2> cv::createBackgroundSubtractorMOG2	(	
int 	history = 500,
double 	varThreshold = 16,
bool 	detectShadows = true 
)	

```

* `history`：用于训练背景的帧数，默认为500帧，如果不手动设置`learningRate`，`history`就被用于计算当前的`learningRate`，此时`history`越大，`learningRate`越小，背景更新越慢；
* `varThreshold`：方差阈值，用于判断当前像素是前景还是背景。一般默认16，如果光照变化明显，如阳光下的水面，建议设为25,36，具体去试一下也不是很麻烦，值越大，灵敏度越低；
* `detectShadows`：是否检测影子，设为true为检测，false为不检测，检测影子会增加程序时间复杂度，如无特殊要求，建议设为false；


```cpp
virtual void cv::BackgroundSubtractor::apply	(	
InputArray 	image,
OutputArray 	fgmask,
double 	learningRate = -1 
)
    
mog->apply(src_YCrCb, foreGround, 0.005);
```

- `image` 源图
- `fmask `前景（二值图像）
- `learningRate` 学习速率，值为0-1,为0时背景不更新，为1时逐帧更新，默认为-1，即算法自动更新；

部分重要参数介绍
`nmixtures`：高斯模型个数，默认5个，最多8个，一般设为5就好，个人测试：设为8个的时候检测效果提升有限，但程序耗时增加；

`backgroundRatio`：高斯背景模型权重和阈值，`nmixtures`个模型按权重排序后，只取模型权重累加值大于`backgroundRatio`的前几个作为背景模型。也就是说如果该值取得非常小，很可能只使用权重最大的高斯模型作为背景(因为仅一个模型权重就大于`backgroundRatio`了)；

`fVarInit`：新建高斯模型的方差初始值，默认15；

`fVarMax`：背景更新过程中，用于限制高斯模型方差的最大值，默认20；

`fVarMin`：背景更新过程中，用于限制高斯模型方差的最小值，默认4；

`varThresholdGen`：方差阈值，用于是否存在匹配的模型，如果不存在则新建一个；

**全部API见**：[MOG2-api](https://docs.opencv.org/3.2.0/d7/d7b/classcv_1_1BackgroundSubtractorMOG2.html)

## 程序参考

```cpp
#include <stdafx>  
#include "opencv2/opencv.hpp"  
#include <vector>  
using namespace cv;
using namespace std;
const int Train = 100;
int main(int argc, char *argv[])
{
	Ptr<backgroundsubtractormog2> mog = createBackgroundSubtractorMOG2(100, 25, false);
	//bgsubtractor->setVarThreshold(20);
	Mat foreGround;
	Mat backGround;
	int trainCounter = 0;
	bool dynamicDetect = true;

	namedWindow("src");
	namedWindow("foreground");

	VideoCapture cap(0);//打开默认的摄像头    
	if (!cap.isOpened())
	{
		return -1;
	}
	Mat src;
	bool stop = false;
	while (!stop)
	{
		cap >> src;

		if (dynamicDetect)
		{
			mog->apply(src_YCrCb, foreGround, 0.005);
			//图像处理过程
			medianBlur(foreGround, foreGround, 3);
			dilate(foreGround, foreGround, Mat(), Point(-1, -1), 3);
			erode(foreGround, foreGround, Mat(), Point(-1, -1), 6);
			dilate(foreGround, foreGround, Mat(), Point(-1, -1), 3);
			imshow("foreground", foreGround);
			if (trainCounter < Train)//训练期间所得结果为不准确结果，不应作为后续
			{
				Mat findc;
				foreGround.copyTo(findc);
				vector<vector<point>> contours;
				cv::findContours(findc, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);

				//targets.clear();
				const int maxArea = 800;
				size_t s = contours.size();
				for (size_t i = 0; i < s; i++)
				{
					double area = abs(contourArea(contours[i]));
					if (area > maxArea)
					{
						Rect mr = boundingRect(Mat(contours[i]));
						rectangle(src, mr, Scalar(0, 0, 255), 2, 8, 0);
						//targets.push_back(mr);
					}
				}
				//string text;					
				char text[50];
				sprintf_s(text, "background training -%d- ...", trainCounter);
				putText(src, text, Point(50, 50), 3, 1, Scalar(0, 255, 255), 2, 8, false);
				//delete[] text;

			}else
			{
				//detects.clear();
				Mat findc;
				foreGround.copyTo(findc);
				vector<vector<point>> contours;
				cv::findContours(findc, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
				const int maxArea = 500;
				size_t s = contours.size();
				RNG rng；
				for (size_t i = 0; i < s; i++)
				{
					double area = abs(contourArea(contours[i]));
					if (area > maxArea)
					{
						Scalar sca_color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
						Rect mr = boundingRect(Mat(contours[i]));
						rectangle(src, mr, sca_color, 2, 8, 0);
						
						//可以对动态目标进行相应操作

					}
				}
				
			}
			trainCounter++;
		}
		
		imshow("src", src);
		
		if (waitKey(30) == 27) //Esc键退出    
		{
			stop = true;
		}
	}
	return 0;
}
```

