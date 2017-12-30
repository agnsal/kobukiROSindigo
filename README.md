# kobukiROSindigo
A kobuki robot ROS package (for both Gazebo and physical robot).


***WORK IN PROGRESS...***

##Instructions:
1. **If you don't have a  catkin workspace, create one:** \
  mkdir -p ~/catkin_ws/src \
  cd ~/catkin_ws/src \
  catkin_init_workspace \
  source catkin_ws/devel/setup.bash
2. **To crate and build the ROS robot package into the workspace:** \
  cd ~/catkin_ws/src \
  catkin_create_pkg kobukiROSindigo std_msgs rospy roscpp \
  cd ~/catkin_ws \
  catkin_make \
  . ~/catkin_ws/devel/setup.bash
3. **To test it:** \
  roscd kobukiROSindigo \
  cat package.xml
4. **(OPTIONAL) To change the dependencies or add a license, as you can see by the file package.xml in this repo:** \
  roscd kobukiROSindigo \
  vi package.xml
5. **To create and build the ROS messages package:** \
  cd ~/catkin_ws/src \
  catkin_create_pkg kobukiROSindigoMSG
  . ~/catkin_ws/devel/setup.bash
6. **To import the code into the 2 new ROS packages:** \
  cd ~/catkin_ws/src \
  git clone https://github.com/agnsal/kobukiROSindigo.git \
  mv kobukiROSindigo/src/* kobukiROSindigo/src/ \
  mv kobukiROSindigo/msg/* kobukiROSindigoMSG/msg/ \
  rm -r kobukiROSindigo 
6. **To build the robot scripts:** \
  cd ~/catkin_ws/src/kobukiROSindigo/src \
  chmod +x senseNode.py 
7. **To run the scripts:** \
  rosrun kobukiROSindigo senseNode.py
