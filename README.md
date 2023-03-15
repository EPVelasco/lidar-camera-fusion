# Description
The code implemented in ROS projects a point cloud obtained by a Velodyne VLP16 3D-Lidar sensor on an image from an RGB camera. The example used the ROS package to calibrate a camera and a LiDAR from [lidar_camera_calibration](https://github.com/ankitdhall/lidar_camera_calibration). In order to have points in a denser cloud, we interpolate the point cloud data by converting the point cloud to a range image and a bilinear interpolation with the armadillo library.

## Interpolated point cloud 
The white dots are the original point cloud of the Velodyne VLP-16 lidar. The colored dots are the interpolated point cloud.
<p align='center'>
<img width="100%" src="/images/lidar_interpolated.png"/>
</p>

## Lidar and camera fusion
<p align='center'>
<img width="80%" src="/images/point_cloud_interpoled.GIF"/>
</p>

## Requisites
- ROS [Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) or [Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)
- [Velodyne](https://github.com/ros-drivers/velodyne) repository
  ```
    cd ~/(your_work_space)/src
    git clone https://github.com/ros-drivers/velodyne.git -b melodic-devel
    cd ..
    catkin_make --only-pkg-with-deps velodyne
  ```
- [PCL](https://pointclouds.org/) (Point Cloud Library) (tested with pcl 1.8)
- [Armadillo](http://arma.sourceforge.net/download.html) (11.0.1 or higher)
  ```
    tar -xvf armadillo-11.1.1.tar.xz
    cd armadillo-11.1.1
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
  ```
## Topics

### Suscribed Topics
*~/pointcloudTopic* Input Point Cloud message. ([sensor_msgs/PointCloud2](http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/PointCloud2.html))

*~/imageTopic* Input image message. ([sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html))

### Published Topics
#### Lidar and camera fusion
*~/points2* Output point cloud interpolated. ([sensor_msgs/PointCloud2](http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/PointCloud2.html))
*~/pcOnImage_image* lidar point cloud projected on input image. ([sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html))
#### Interpolated point cloud
*~/pc_interpoled* Output point cloud interpolated. ([sensor_msgs/PointCloud2](http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/PointCloud2.html))
## Clone repository
```
    cd ~/catkin_ws/src
    git clone https://github.com/EPVelasco/lidar_camera_fusion.git
    cd ..
    catkin_make --only-pkg-with-deps lidar_camera_fusion
```

## Ros Launch
#### Lidar and camera fusion
```
  roslaunch lidar_camera_fusion vlp16OnImg.launch 
```
#### Interpolated point cloud
```
  roslaunch lidar_camera_fusion interpolated_vlp16.launch
```

## Applications
Detection and depth estimation for domestic waste in outdoor environments by sensors fusion. [Preprint](https://arxiv.org/abs/2211.04085)

## Citation
Application
```
@article{paez2022detection,
  title={Detection and depth estimation for domestic waste in outdoor environments by sensors fusion},
  author={P{\'a}ez-Ubieta, Ignacio de L and Velasco-S{\'a}nchez, Edison and Puente, Santiago T and Candelas, Francisco A},
  journal={arXiv preprint arXiv:2211.04085},
  year={2022}
}
```
Code
```
@misc {EPVelasco_lidar, 
  author = {Edison Velasco}, 
  title = {Lidar and camera fusion}, 
  year = {2022}, 
  editor = {GitHub}, 
  revista = {GitHub Repository},
  url ={github.com/EPVelasco/lidar-camera-fusion}, 
}  
```
