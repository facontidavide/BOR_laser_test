# Convert LaserScan Scanner to PointCloud2

Data acquired from a 2D Laser in ROS are expressed in polar coordinates, relative to the position of the laser itself.
We want to convert laser data into cartesian coordinates (3D), relative to a fixed frame in the world.

**IMPORTANT**: there might be ROS packages which perform the trasformation required by this assignment. You are supposed to do you own implementation (reinvetn the wheel) to show that you understand the math and you are able to implement it. 

DO NOT use or copy 3rd party code.

To test your implementation, use the included file **laser.bag**.

You need to create a ROS node to perform the convertion.

- Subscribe to a topic of type [sensor_msgs::LaserScan](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html), 
   to obtain the laser data.

- Use [TF2](http://wiki.ros.org/tf2/Tutorials) to provide the tansform between the fixed frame **/odom_combined** and the laser (**base_laser_link**).

- Transform the raw data of the LaserScan message into a pointcloud with 
  type [pcl::PointCloud< pcl::PointXYZI >](http://pointclouds.org/documentation/tutorials/basic_structures.php#basic-structures).
  You are encouraged to use either [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) or 
  [tf2::Transform](http://docs.ros.org/jade/api/tf2/html/classtf2_1_1Transform.html).

- Convert **pcl::PointCloud<pcl::PointXYZI>** into [sensor_msgs::PointCloud2](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html)
  as described [here](http://ros-developer.com/2017/08/03/converting-pclpclpointcloud2-to-pclpointcloud-and-reverse/) 
  and publish it.
  
 ##  Requirements and suggestions
  
 - Fork this repository. Your forked repository **must be private**. You may (should?) commit intermediate results.
 - Once you have done, send an email.
 - Write code that is readable and well organized.
 - You are encouraged to have a good separation between the ROS node and the core algorithm.
 - You can use `rosbag play` and **RViz** to test if your implementation is correct (you are expected to know how to do this).
  
 ## Optional : optimization opportunity
 
 Suppose that the following fields of LaserScan are **constant** (we can read them from the first LaserScan message that we receive)
 
 - angle_min
 - angle_max
 - angle_increment
 
 Given this assumption, it **is** possible to **considerably** speed up your application in terms of CPU usage, AKA reduce the amount of computation. How?
 
 Either implement the solution (preferably) or, if you run out of time, give a very detailed description of how you would do it.
 
