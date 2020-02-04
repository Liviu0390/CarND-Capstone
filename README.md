This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the "uWebSocketIO Starter Guide" found in the classroom (see Extended Kalman Filter Project lesson).

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

### Other library/driver information
Outside of `requirements.txt`, here is information on other driver/library versions used in the simulator and Carla:

Specific to these libraries, the simulator grader and Carla use the following:

|        | Simulator | Carla  |
| :-----------: |:-------------:| :-----:|
| Nvidia driver | 384.130 | 384.130 |
| CUDA | 8.0.61 | 8.0.61 |
| cuDNN | 6.0.21 | 6.0.21 |
| TensorRT | N/A | N/A |
| OpenCV | 3.2.0-dev | 2.4.8 |
| OpenMP | N/A | N/A |

We are working on a fix to line up the OpenCV versions between the two.



### Writeup

# Waypoint Updater

The purpose of the Waypoint Updater node is to update the target velocity of each waypoint, in order for the car to decelerate or to stop smoothly.

The topics to which the Waypoint Updater node subscribes, are:
'/current_pose' topic, which is used to get the vehicles current position.
'/base_waypoints' topic, used to get the waypoints ahead of the vehicle position.
'/traffic_waypoint' topic is used to receive the waypoint information from the Traffic Light Detector node. Traffic waypoint is the waypoint where the car is expected to stop.

The main functionality of the node is to update the the target speed of the car depending on the traffic lights color received from the '/traffic_waypoint' topic.
If the color of the traffic light is green, no modifications are made.
If the color detected is red, the car will smoothly slow down and come to a complete stop in an interval declared as stop_distance + break_distance.
Also, if the traffic light color changes the following rules apply:
If the traffic light changes from red to green, the target velocity is set up to 25 MPH in the waypoints ahead of the vehicle.
If the traffic light changes from green to red and the car was accelerating in the stopping distance, the target speed is set to zero, so the car will stop before the traffic light.

The Waypoint Updater node's output is published on the '/final_waypoints' topic.
The output are 60 waypoints ahead of the car, containing the target velocities, based on the traffic lights color.

# DriveByWire Node

This node is responsible for steering the car and speed control. The node subscribes to '/twist_cmd', '/current_velocity', '/vehicle/dbw_enabled' and '/current_pose'.

In '/twist_cmd' topic, twist commands are published by the waypoint follower node.
The '/current_velocity' topic is used to determine the linear velocity of the car and provide the data to the controller.
For manual controll of the car, the '/vehicle/dbw_enabled' topic is used. If true, the throttle, steer and brake are published to the respective topics.
For steering, the yaw_controller is used and for the throttle a PID controller is used. 
A subscriber is used for '/current_pose' topic to get the current angle.

In the control method in the twist_controller.py, we implemented an adaptive braking system depending on the difference between the target velocity and the current velocity.
The brake value is smaller as the difference in velocities is smaller, and greater if the difference is bigger. The throttle value is set to zero when brakes are applied.

The DBW_node publishes throttle, brake and steering commands for the car, if dbw_is_enabled flag is true.


# Traffic Light Detection and Classification

For proccessing, our team used 2 trained SSD Inception V2 models for our Capstone project:
	1 SSD model for real-world data
	1 SSD model for simulator data

For training, we used 200 images for each model that were manually created by us.

The Traffic Light Detection node takes data from the '/image_color', '/current_pose' and '/base_waypoints' topics.
The '/image_color' topic provides an image stream from the car's camera, which is used to determine the traffic light color.
The '/current_pose' topic is used to determine the current position of the car.
And finally, the '/base_waypoints' topic provides a complete list of the course waypoints.

The traffic light state is given by the get_light_state mehod, which is called by process_traffic_lights method.
The process_traffic_lights method finds the nearest traffic light to the car (closest stop line waypoint index), process the image and return the line waypoint index and the traffic light state.

The logic for the traffic light detector is implemented in the loop() method. 
If the light state did not changed more than the threshold, the last stable state is used. Otherwise, the new state is used.

The tf detector node publishes to the '/traffic_waypoint' topic the traffic light waypoint where the car needs to stop.


# Team members
| Name | Email  |
| :-----------: |:-------------:|
| Alexandru Dan Frujina | alexandru.frujina@wipro.com |
| Alexandru Moruz | moruz.alexandru@wipro.com |
| Bogdan Valentin Balan |  bogdan.balan@wipro.com |
| Dragos Ronald Rugescu |  dragos.rugescu@wipro.com |
