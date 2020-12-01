# Robile Mobotics

EECE 5550 Mobile Robotics  
Mohammadreza Sharif

## Setup

```bash
cd ~
git clone git@github.com:sharif1093/apriltag_gazebo_model_generator.git
cd ~/apriltag_gazebo_model_generator/ar_tags/scripts
./generate_markers_model.py -i ../36h11_sample -s 200 -w 50
```

```bash
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
sudo apt install ros-noetic-turtlebot3-msgs ros-noetic-image-transport-plugins
sudo apt-get install imagemagick
```

```bash
git submodule update --init --recursive
```

Additional ROS packages:

``` bash
sudo apt install ros-noetic-gmapping ros-noetic-map-server ros-noetic-amcl ros-noetic-move-base ros-noetic-dwa-local-planner
```

Add explore_lite but exclude multrobot_map_merge
```
catkin config --blacklist multirobot_map_merge
```


## Resources
[https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/]()
[https://aws.amazon.com/blogs/robotics/hospital-world-simulating-robot/](AWS HospitalWorld Blog Post)
[http://wiki.ros.org/navigation/Tutorials/RobotSetup](move_base params: ROS navigation wiki)