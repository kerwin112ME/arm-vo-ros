# arm-vo-ros
A modified version of zanazakaryaie's [arm-vo](https://github.com/zanazakaryaie/ARM-VO) 
Added features include:   
1. ROS wrapper
2. 2D version (assume pitch = roll = 0, z = 0)  

# How to install
Clone the repository to your catkin workspace / src   
``` git clone https://github.com/kerwin112ME/arm-vo-ros.git ```   
catkin make the package   
``` cd .. ```   
``` catkin_make ```   

Note: This package is only for arm cpu.   

# How to run
Set your parameters in yaml file, include the param.yaml in the launch file.    
``` roslaunch arm_vo test.launch ```    
Run your rosbag in another terminal.    
``` rosbag play [your_bag].bag ```    
