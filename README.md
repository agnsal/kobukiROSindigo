# kobukiROSindigo
A kobuki robot ROS package (for both Gazebo and physical robot).

## Instructions:
1. **If you don't have a  catkin workspace, create one:** \
  mkdir -p ~/catkin_ws/src \
  cd ~/catkin_ws/src \
  catkin_init_workspace \
  source catkin_ws/devel/setup.bash
2. **To crate and build the ROS robot package into the workspace:** \
  cd ~/catkin_ws/src \
  catkin_create_pkg kobukiROSindigo std_msgs rospy roscpp \
  mkdir kobukiROSindigo/src \
  mkdir kobukiROSindigo/msg \
  cd ~/catkin_ws \
  catkin_make \
  . ~/catkin_ws/devel/setup.bash
3. **To test it:** \
  roscd kobukiROSindigo \
  cat package.xml
5. **To import the code into the ROS package and build all its dependencies:** \
  cd ~/catkin_ws \
  git clone https://github.com/agnsal/kobukiROSindigo.git \
  mv kobukiROSindigo/src/* src/kobukiROSindigo/src/ \
  mv kobukiROSindigo/msg/* src/kobukiROSindigo/msg/ \
  rm src/kobukiROSindigo/package.xml \
  rm src/kobukiROSindigo/CMakeLists.txt \
  mv kobukiROSindigo/package.xml src/kobukiROSindigo/ \
  mv kobukiROSindigo/CMakeLists.txt src/kobukiROSindigo/ \
  rm -r kobukiROSindigo \
  catkin_make
6. **To build the robot scripts:** \
  cd ~/catkin_ws/src/kobukiROSindigo/src \
  chmod +x senseNode.py 
7. **To run the scripts:** \
  rosrun kobukiROSindigo senseNode.py
