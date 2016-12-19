# delta_final_project

This repository is built on wsnewman¨s learning ROS, available at https://github.com/wsnewman/learning_ros

To start, clone the learning ROS repo, and substitute the Part6/coordinator/launch in learning_ros with our Part6/coordinator/launch directory.

Then, substitute the pcl_utils and object_grabber directories under Part3 with ours accordingly.

Example usage:

 ！- On a machine connected to the Baxter robot:
  > baxter_master
  > roscd
  > catkin_make
  > roslaunch freenect_launch freenect.launch

 ！- new terminal tab
  > roslaunch pcl_utils finder.launch

 ！- new terminal tab
  > roslaunch coordinator coord_vision_manip2.launch

 ！- new terminal tab
  > rosrun object_finder example_object_finder_action_client

 Place block / cube on the stool, following the prompt, and see the Baxter drop it down!


