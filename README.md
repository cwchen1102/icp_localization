# ICP_Localization
Iterative closest point (ICP) is an algorithm employed to minimize the difference between two clouds of points, and widely used in localization of robot.

In this document, you should have two point cloud data
1 .Map(map.pcd)
2. Bag(obtained from your sensor)
And you are going to use these point cloud data to localize the car by ICP algorithm.

## Requirement
1. Use PCL Library to perform ICP algorithm, and localize the car
2. Publish tf to visualize scan matching process

## Steps
1. Load map 
2. Subscribe point cloud 
3. Implement ICP using PCL 
4. Publish tf

## tf
ROS provides a package tf to maintain the relationship between different coordinate frame. After you successfully publish the tf, it would be able to see the current scan and map in the same frame.
For more implementation details, see this
http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20(C%2B%2B)

## Notes
To ensure your bag time and ROS time are the same, you may need to type the following command to play bag: 
```c++
rosparam set use_sim_time true
rosbag play -r 0.3 sdc_hw5.bag --clock
```

## Video
https://www.youtube.com/watch?v=joHdoiLyQHk&t=4s


