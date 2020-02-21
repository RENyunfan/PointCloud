# SS Lidar

> Solid State Lidar
>
> Our goal is to design a high efficiency algorithm to label the position of things in the 3D world. First we need to cluster/recognized all things. Then we are going to use geometry method to locate specific things in the real world.
>
> 
>
> by Yunfan REN
>
> renyunfan@berkeley.edu



```mermaid
gantt
    dateFormat  YYYY-MM-DD
    title Solid State Lidar

    section PCL
    Install PCL            		:done,    des1, 2020-02-11,1d
    PCL groundSeg               :done,  des2, 2020-02-14, 3d
    PCL Noise Filter            :active         des3, after 2020-02-14, 5d
    3D-2D						:done	des3, after des2,3D
   

    section Learning
    SqueezeSeg						    :L1, 2020-02-11,20d
    PointNet					        :L2, after L1, 2d

    section Geometrary
    Rigidbody Transformation              :active, a1, 2020-03-01, 5d


  

```

# 0 Idea

## How to recognize object

### 1 RGB-D

Use RGB image and CNN to segment objects, and then use geometry methods to find the associate location of specific object.

### 2 3D learning method

Use 3D cnn to segment objects directly and get the depth data. Use basic geometry method to get the location.

## Make the most of static scenes

### 1 Filter of the ground

[基于几何特征的地面点云分割](https://zhuanlan.zhihu.com/p/34815976)

### 2 Static object filtering

It takes more time to initialize the scene, semantically segment the scene in advance, and then divide and conquer the problem of target recognition.

# 1 Process

## Feb

### 11 SqueezeSeg

* Get the date.
* Try some learing method to segment the car from the point cloud image.



Modified and reproduced the image segmentation algorithm in SqueezeSeg, using existing models to segment the point cloud image.

<img src="./source/image/squeezeseg.png" alt="test" style="zoom:50%;" />

### 14 Ground filter

[Notes](./Notes/Feb_14_Voxel_Grid_Filter.md)


<center>
    <img style="border-radius: 0.3125em;
    box-shadow: 0 2px 4px 0 rgba(34,36,38,.12),0 2px 10px 0 rgba(34,36,38,.08);" 
    src="./source/image/ground.png">
    <br>
    <div style="color:orange; border-bottom: 1px solid #d9d9d9;
	display: inline-block;
    color: #000000;
    padding: 2px;">From left to right: Ground, Original, No-ground-Seg</div>
</center>

### 15  Voxel Grid Filter

<center>
    <img style="border-radius: 0.3125em;
    box-shadow: 0 2px 4px 0 rgba(34,36,38,.12),0 2px 10px 0 rgba(34,36,38,.08);" 
    src="./source/image/Voexl.png">
    <br>
    <div style="color:orange; border-bottom: 1px solid #d9d9d9;
	display: inline-block;
    color: #000000;
    padding: 2px;">Voexl seg (1,1,1)</div>
</center>

**Idea**

```mermaid
graph LR
A(Point Cloud)
B(Two-dimensional)
C(Digital Image Process)
D(Turn to 3D)
A-->B
B-->C
C-->D

```

### 17 3D to 2D

[Notes of 3D to 2D](./Notes/Feb_17_Birdseye.md)

![](./source/image/3D22D.png)

### 18 GMM background seg

[Feb_18_Gaussian Mixed Model.md](./Notes/Feb_18_Gaussian Mixed Model.md)

<center>
    <img style="border-radius: 0.3125em;
    box-shadow: 0 2px 4px 0 rgba(34,36,38,.12),0 2px 10px 0 rgba(34,36,38,.08);" 
    src="README.assets/Gaussian.png">
    <br>
    <div style="color:orange; border-bottom: 1px solid #d9d9d9;
	display: inline-block;
    color: #000000;
    padding: 2px;">From left to right: Original, GMM+Medium Filter, Only GMM</div>
</center>

## 20 GMM+pcl/filter 

[Notes](./Notes/Feb_20_GMM_pclfilter.md)

![](./Notes/Feb_20_GMM_pclfilter.assets/2D3D.png)


# 2 To Do List

* Learn the octree
* Learn to use PCL and basic point cloud operation
* Learn some point cloud cluster algorithm.



# 3 Question



# 4 Resources

[Out kernel octree](https://zhuanlan.zhihu.com/p/103701375)

[ROS & PCL](https://zhuanlan.zhihu.com/p/103700110)

[Segmentation in PCL](https://zhuanlan.zhihu.com/p/103700893)