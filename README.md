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
echo "export export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
sudo apt install ros-noetic-turtlebot3-msgs ros-noetic-image-transport-plugins
sudo apt-get install imagemagick
```

```bash
git submodule update --init --recursive
```

Additional ROS packages:  
``` bash
sudo apt install ros-noetic-gmapping ros-noetic-map-server ros-noetic-frontier-exploration ros-noetic-navigation-stage
```

## Resources
[https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/]()
