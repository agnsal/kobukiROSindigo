# kobukiROSindigo
A kobuki robot ROS package (for both Gazebo and physical robot).


***WORK IN PROGRESS...***

##Instructions:
1. **To crate and build the ROS package:** \
  cd ~/catkin_ws/src \
  catkin_create_pkg kobukiROSindigo std_msgs rospy roscpp \
  cd ~/catkin_ws \
  catkin_make \
  . ~/catkin_ws/devel/setup.bash
2. **To test it:** \
  roscd kobukiROSindigo \
  cat package.xml
3. **(OPTIONAL) To change the dependencies OR add a license, as you can see by the file package.xml in this repo:** \
  vi package.xml
4. **To import this code into the ROS package:** \
  mkdir scripts \
  git clone https://github.com/agnsal/kobukiROSindigo.git \
  mv kobukiROSindigo/scripts/* scripts/ \
  rm -r kobukiROSindigo
