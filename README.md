# camera-laser-calibration
 Author: Nico

# overview
This package is used to calculate the translation and rotation matrix between the camera and laser coordinate.
The QR code is used as a marker. The center point of the QR code is detected as a 2D point in the image and a 3D point in the laser coordinate. The PnP method is used to get the relation between the two coordinates. Actually you can use any other markers. They will give you good results as well.
The following image shows the image and laser fusion result:
![](https://github.com/NicoChou/camera-laser-calibration/raw/master/screenshots/1.png)
# prerequisites
## 1. ROS
[ROS](http://wiki.ros.org/indigo/Installation/Ubuntu) is used to obtain the image and laser message.
## 2. Marker
Here we use the QR code as the marker. The size of the marker is 80*80 cm.

Marker

<img src="https://github.com/NicoChou/camera-laser-calibration/raw/master/screenshots/2.png" width="50%" height="50%">

# Test data
The test data can be found [here](https://drive.google.com/open?id=1D8UdXUjMLe6_LDUoY68yLEMGVh87Yrv2). This rosbag has two topics:
image topic: /camera/image_color 
laser topic: /velodyne32/velodyne_points
Run the bag file:
```
$rosbag play 2016-12-22-14-03-09.bag -l
```
Follow the "Usage" step to test the package.
# Usage 
## 1. camera calibtation 
Ignore this step if you are using the test data.
```
$rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.061 image:=/camera/image_color camera:=/camera
```
Change the size/square/image/camera with your own parameters. Write the camera calibration result in the cam_laser_calib/src/solvepnp/param/calib.yml file.
## 2. Get the point pair of the QR code center
### 2.1 build and run
Create a ros workspace named cam_laser_calib.
``` 
$cd cam_laser_calib/src
$git clone git@github.com:NicoChou/camera-laser-calibration.git
$cd ..
$catkin_make
$roslaunch  cam_laser_calib calibration.launch
```
Change the "onlyDrawPointsColor" parameter value in launch file to false. Replace the point cloud and image topics with your own topic name.
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
### 2.2 Choose rectangle cutting area of  point cloud
The rqt_reconfigure is used to dynamic config the rectangle cutting area of point clouds.
```
$rosrun rqt_reconfigure rqt_reconfigure 
```
rqt_reconfigure

![rqt_reconfigure](https://github.com/NicoChou/camera-laser-calibration/raw/master/screenshots/3.png)

Before cut the point cloud

<img src="https://github.com/NicoChou/camera-laser-calibration/raw/master/screenshots/4.png" width="50%" height="50%">

After cut the point cloud

<img src="https://github.com/NicoChou/camera-laser-calibration/raw/master/screenshots/5.png" width="50%" height="50%">

Estimated plane

<img src="https://github.com/NicoChou/camera-laser-calibration/raw/master/screenshots/6.png" width="50%" height="50%">

Adjust the parameter and wirite them into the cam_laser_calib/src/camLaserCalib/cfg/cam_laser_calib.cfg file. 
Shutdown the program and recompile the package.
```
$cd cam_laser_calib/
$catkin_make 
```
### 2.3 Get the points pair of QR code center
```
$roslaunch  cam_laser_calib calibration.launch
```
After this step, write the point pairs of QR code center under both laser coordinate and image coordinate to cam_laser_calib/src/solvepnp/imageCloudPoints.txt file.
## 3. Calculate the calibration matrix
Use the solvePnP method in openCV to get the calibration matrix.
```
$cd cam_laser_calib/src/solvepnp/build
$make
$../bin/solvepnp
```
Copy the T matrix shown in the terminal to cam_laser_calib/src/solvepnp/param/calib.yml file to replace the CameraExtrinsicMat.
## 4. Use the calibration matrix to draw point clouds with image color
At this step, the onlyDrawPointsColor parameter in "calibration.launch" file can be changed to true, and then you will be able to get the color point cloud as the first figure.



