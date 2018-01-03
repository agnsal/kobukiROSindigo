# kobukiROSindigo
A kobuki robot ROS package (for both Gazebo and physical robot) that uses SWI-Prolog to take decisions. 

## Instructions:
1. **Install SWI-Prolog:** \
  apt-get install autoconf \
  git clone https://github.com/SWI-Prolog/swipl-devel.git \
  cd swipl-devel/package
  chmod u+x configure
  **To configure the source with shared library enabled:** ./configure --prefix=/usr --enable-shared \
  make \
  sudo make install \
  cd packages/clpqr \
  ./configure --prefix=/usr --enable-shared $ make && sudo make install \
  sudo ln -s /usr/lib/pl-5.6.34/lib/i686-linux/libpl.so.5.6.34 /usr/lib/libpl.so  \
2. **Install the Python needed library:** \
  git clone https://github.com/yuce/pyswip.git \
  tar xzvf pyswip
3. **If you don't have a  catkin workspace, create one:** \
  mkdir -p ~/catkin_ws/src \
  cd ~/catkin_ws/src \
  catkin_init_workspace \
4. **To crate and build the ROS robot package into the workspace:** \
  cd ~/catkin_ws/src \
  catkin_create_pkg kobukiROSindigo std_msgs rospy roscpp \
  mkdir kobukiROSindigo/src \
  mkdir kobukiROSindigo/msg \
  cd ~/catkin_ws \
  catkin_make \
  source devel/setup.bash
5. **To test it:** \
  roscd kobukiROSindigo \
  cat package.xml
6. **To import the code into the ROS package and build all its dependencies:** \
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
7. **To build the robot scripts:** \
  cd ~/catkin_ws/src/kobukiROSindigo/src \
  chmod +x senseNode.py \
  chmod +x thinkNode.py \
  chmod +x actNode.py 
8. **To run the scripts on the robot or on the simulation (that have to have been launched):** \
  rosrun kobukiROSindigo senseNode.py \
  rosrun kobukiROSindigo thinkNode.py \
  rosrun kobukiROSindigo actNode.py
