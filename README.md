# Convert LaserScan Scanner to PointCloud2

Data acquired from a 2D Laser in ROS are expressed in polar coordinates, relative to the position of the laser itself.
We want to convert laser data into cartesian coordinates (3D), relative to a fixed frame in the world.

To test your implementation, use the included file **laser.bag**.

You need to create a ROS node to perform the convertion.

- Subscribe to a topic of type [sensor_msgs::LaserScan](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html), 
   to obtain the laser data.

- Use [TF2](http://wiki.ros.org/tf2/Tutorials) to provide the tansform between the fixed frame **/odom_combined** and the laser (**base_laser_link**).

- Transform the raw data of the LaserScan message into a pointcloud with 
  type [pcl::PointCloud<pcl::PointXYZI>](http://pointclouds.org/documentation/tutorials/basic_structures.php#basic-structures).
  You are encouraged to use either [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) or 
  [tf2::Tranform](http://docs.ros.org/jade/api/tf2/html/classtf2_1_1Transform.html).

- Convert **pcl::PointCloud<pcl::PointXYZI>** into [sensor_msgs::PointCloud2](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html)
  as described [here](http://ros-developer.com/2017/08/03/converting-pclpclpointcloud2-to-pclpointcloud-and-reverse/) 
  and publish it.
  
 ##  Requirements and suggestions
  
 - Fork this repository. Your forked repository **must be private**. You may (should?) commit intermediate results.
 - Write code that is readable and well organized.
 - You are encouraged to have a good separation between the ROS node and the core algorithm.
 - You can use `rosbag play` and **RViz** to test if your results are correct.
  
 ## Optional : optimization opportunity
 
 Suppose that the following fields of LaserScan are **constant** (we can read them from the first LaserScan message)
 
 - angle_min
 - angle_max
 - angle_increment
 
 Given this assumption, it is possible to considerably speed up (in terms of CPU usage) our application. How?
 
 Either implement the solution (preferably) of give a very detailed description of how you would do it.
 
