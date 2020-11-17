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

```bash
rosrun image_transport republish compressed in:=/camera/rgb/image_raw _image_transport:=compressed raw out:=/camera/rgb/image_raw
```