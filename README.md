# Readme

## Preparation
- ubuntu 20.04
- ros melodic
- eigen

### eigen3 install
~~~
git clone https://github.com/eigenteam/eigen-git-mirror
cd eigen-git-mirror
mkdir build
cd build
cmake ..
sudo make install
sudo cp -r /usr/include/eigen3/Eigen /usr/include 
~~~
## compile
~~~
cd ros_demo01
catkin_make --pkg common
catkin_make
~~~

## use
~~~
roscore

~~~
