# camera-laser-calibration
 Authors: Nico

#overview
This package is used to calculate the translation and rotation matrix between camera and laser coordinate.
I use the QR code board as the marker,detect the center point pair of the QR code as 2D point in image and 3D points in laser coordinates. Then use the PnP method to get the relation between the two coordinates.Actualy you can use any other marker .It will also give you good result.
Image shows the image and laser fuse result:
![](https://github.com/NicoChou/camera-laser-calibration/raw/master/screenshots/1.png)
#prerequisites
##1 ROS
We use [ros](http://wiki.ros.org/indigo/Installation/Ubuntu) to get the image and laser message.
##2 Marker
Here we use the QR code as the marker.The size of the marker is 80*80 cm.

Marker

<img src="https://github.com/NicoChou/camera-laser-calibration/raw/master/screenshots/2.png" width="50%" height="50%">

#Test data
This rosbag have two topic:
image topic:/camera/image_color 
laser topic: /velodyne32/velodyne_points
Run the bag file:
```
$rosbag play 2016-12-22-14-03-09.bag -l
```
And follow the "How to use"step, you can test the package.
#How to use 
##1 camera calibtation 
If you use the test data ， you can ignore this step.
```
$rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.061 image:=/camera/image_color camera:=/camera
```
You shoude change the size/square/image/camera with your own parameters.Write the camera calibration result in the cam_laser_calib/src/solvepnp/param/calib.yml file.
##2 Get the points pair of QR code center
### 2.1 build and run
```
$cd cam_laser_calib/
$catkin_make
$roslaunch  cam_laser_calib calibration.launch
```
Here we shoud notice the parameter in launch file,change the "onlyDrawPointsColor" value to false.Replace the point cloud and image topic with your own topic name.
```
<launch>

	<node pkg="cam_laser_calib" type="cam_laser_calib_node" name="cam_laser_calib" args="$(find cam_laser_calib)/../solvepnp/imageCloudPoints.txt $(find cam_laser_calib)/../solvepnp/param/calib.yml" output="screen">

		    <param name="strSub_pc2" type="string" value="/velodyne32/velodyne_points"/>

	   		<param name="strSub_img" type="string" value="/camera/image_color"/>

	  	 	<param name="onlyDrawPointsColor" type="bool" value="false"/>

		    <param name="DistanceThreshold" type="double" value="0.05"/>

	</node>

</launch>
```
###2.2 Choose rectangle cut area of  point cloud
We use rqt_reconfigure to dynamic config the rectangle cut area of  point cloud.
```
$rosrun rqt_reconfigure rqt_reconfigure 
```
rqt_reconfigure

![rqt_reconfigure](https://github.com/NicoChou/camera-laser-calibration/raw/master/screenshots/3.png)

Before cut the point cloud

![Before cut the point cloud](https://github.com/NicoChou/camera-laser-calibration/raw/master/screenshots/4.png)

After cut the point cloud

<img src="https://github.com/NicoChou/camera-laser-calibration/raw/master/screenshots/5.png" width="150%" height="150%">
Estimated plane

<img src="https://github.com/NicoChou/camera-laser-calibration/raw/master/screenshots/6.png" width="180%" height="180%">

After adjust the parameter,wirite it in the  cam_laser_calib/src/camLaserCalib/cfg/cam_laser_calib.cfg file and shutdown the program. Recompile the package.
```
$cd cam_laser_calib/
$catkin_make 
```
###2.3 Get the points pair of QR code center
```
$roslaunch  cam_laser_calib calibration.launch
```
After this step, we write the point pairs of QR code center in laser coordinates and  image coordinates to cam_laser_calib/src/solvepnp/imageCloudPoints.txt file.
##3 Calculate the calibration matrix
We use solvePnP method in openCV to get the calibration matrix.
```
$cd cam_laser_calib/src/solvepnp/build
$make
$../bin/solvepnp
```
After this step , copy the T matrix in terminal to cam_laser_calib/src/solvepnp/param/calib.yml file. Replace the CameraExtrinsicMat.
##4 Using calibration matrix to draw point cloud with image color
At this step we could change the onlyDrawPointsColor parameter in "calibtation.launch" file to true.Got the color point cloud as the first figure.



