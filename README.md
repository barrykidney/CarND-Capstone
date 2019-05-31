# Capstone Project: Programming a Real Self-Driving Car

## Team: “Who want to build a Team?”

| Name            | E-mail                  |
| :-------------- | :---------------------- |
| Barry Kidney    | barrykidney@hotmail.com |
| Andreas Boerzel | andreas.boerzel@gmx.de  |
| Srujan K        | srujank22@gmail.com     |
| Tomohiro Yamada | yamady0711@yahoo.co.jp  |

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car.  For more information about the course, see the project introduction [here](https://eu.udacity.com/course/self-driving-car-engineer-nanodegree--nd013

-------------------------------------------------------
## 1. Overview of the Capstone Project  
Programming a Real Self-Driving Car project is the final capstone project of the Udacity Self-Driving Car Engineer Nanodegree.  
For this project, our team has developed software to control Carla, Udacity’s actual Self-Driving Car.  
We have written ROS nodes to implement core functionality of the autonomous vehicle system, including traffic light detection & classification, control, and waypoint updating.  

The following is a system architecture diagram showing the ROS nodes and topics used in the project. The ROS nodes and topics shown in the diagram are described briefly in the Code Structure section below.
![Code Structure](imgs/Code-structure.png)

-------------------------------------------------------
## 2. Code Structure
Below is a brief overview of the repo structure, along with descriptions of the ROS nodes. The following ROS pacakges are contained entirely within the  `CarND-Capstone/ros/src/` directory. 



#### 2-1. Planning Subsystem / Waypoint Updater Node
![Waypoint Updater](imgs/Waypoint-updater-node.png)
The package `ros/src/waypoint_updater/` contains the waypoint updater node: `waypoint_updater.py`. The waypoint updater node is responsible for selecting a predetermined number of waypoints ahead `LOOKAHEAD_WPS` of the vehicle that describe the desired route and publish's the list with target velocities to the `/final_waypoints` topic.  The waypoint updater subscribes to the `/base_waypoints` topic which consists of a provided set of waypoints marking the path the vehicle should follow around the course, The `/current_pose` topic which consists of information describing the vehicles current state including coordinates, velocity, yaw, etc. and the `/traffic_waypoint` topic which we will describe in a later section.

The waypoint updater's logic is as follows. The closest waypoint to the vehicle is obtained, by passing the `current_pose` as a parameter to the `self.get_closest_waypoint_idx()` function. This function returns the closest base waypoint to the vehicles current coordinates. It then adds the closest waypoint and the subsequent `n`  waypoints to a list. The target velocity property of each waypoint are then updated and the list is published to the `/traffic_waypoint` topic.




#### 2-2. Control Subsystem / DBW(Drive-By-Wire) Node
![DBW](imgs/dbw-node.png)
The Control subsytem of Carla is equipped with “DBW(Drive-By-Wire) node” which publishes the vehicle's throttle, steering and brakes commands based on current velocity and target velocity provided by “Waypoint Follower node” .  

Once messages are published to `/final_waypoints` by The 'waypoint updater' Node, the vehicle's 'waypoint follower' Node (provided by Udacity) will publish twist commands to the `/twist_cmd` topic. The DBW node (`dbw_node.py`) subscribes to `/twist_cmd` and uses various controllers to provide appropriate throttle, brake, and steering commands. In case a safety driver takes control of the car during testing, DBW node stops controlling the car based on the topic: `/vehicle/dbw_enabled`.


The DBW node also subscribes to the `/current_velocity` topic. Prior to the calculation of control output, current velocity info is passed through a Low Pass filter to filter the jitter/noise from simulator/actual car.  

1. Throttle control is done by PID controller which takes in the current and target velocity and calculates the adjustment to be published to `/vehicle/throttle_cmd`. The parameters were tweaked by trial and error based on the policy below.  
- Proportional feedback is the main factor of PID control and the Kp value can be relatively large without reducing the control stability margin, generally speaking.   
- Integral feedback adjusts for offset, therefore it is not required to change dynamically and a small Ki value is sufficient for this purpose.  
- Derivative feedback is useful for mitigating sudden state change or quick motion but can lead to an unstable state, therefore the Kd value should be non-zero but a relatively conservative setting is preferable.  

2. Steering Control calculates target steering angle based on the linear and angular velocities using the vehicle’s steering ratio and wheel-base data, it is published to the `/vehicle/steering_cmd` topic.  

3. Braking Control calculates target deceleration torque based on vehicle’s velocity error, vehicle weight, and wheel radius, it is  published to the `/vehicle/brake_cmd` topic.  



#### 2-3. Perception Subsystem / Traffic Light Detection Node

![Traffic Light Detection](D:/Udacity/Self_driving_engineer/CarND-Capstone/imgs/tl-detection-node.png)
Once the vehicle is able to process waypoints, generate steering and throttle commands, and traverse the course, it will also need stop for obstacles. Traffic lights are the obstacle that we'll focus on. Please follow the link below which describes the details of Traffic Light Detection Node we developed. -> [Link](https://github.com/aboerzel/Traffic-Light-Detection "Link")  



#### 2-4. Planning Subsystem with traffic light detection

A stop line is detected if the `/traffic_waypoint` topic publishes a stop line index, this occurs if the traffic signal is in front of the vehicle, within range and has been classified as being in a red or a yellow state. 

When this occurs the `base_waypoints` and the`closest_idx` parameters are passed to the `self.decelerate_waypoints` function. This function calculates the exact waypoint where the vehicle should stop, so that its front wheels are close to the stop line without crossing it. Next the velocity at each waypoint is calculated, this is done using the pre-set constant value of maximum deceleration (MAX_DECEL) and the number of waypoints the vehicle generates (LOOKAHEAD_WPS). The velocity has to be reduced in such a way that the vehicle does not come to an abrupt halt thus violating the maximum allowed jerk value specified in the project guidelines. This reduction in velocity is calculated with the formula _vel = math.sqrt(2 * MAX_DECEL * dist)+ (i / LOOKAHEAD_WPS)_. As the velocity of the vehicle is represented by its waypoints, the returned list of values are the new decelerated waypoint velocity values that describe the vehicle slowing, then coming to stop before the stop line.



-------------------------------------------------------
## 3. How to install and execute the code
Please use **one** of the two installation options, either native **or** docker installation.

### 3-1. Native Installation

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

### 3-2. Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### 3-3. Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### 3-4. Usage

1. Clone the project repository
```bash
git clone https://github.com/barrykidney/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
sudo apt-get update
sudo apt-get install -y ros-kinetic-dbw-mkz-msgs
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

### 3-5. Real world testing
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

### 3-6. Other library/driver information
Outside of `requirements.txt`, here is information on other driver/library versions used in the simulator and Carla:

Specific to these libraries, the simulator grader and Carla use the following:

|               | Simulator |  Carla  |
| :-----------: | :-------: | :-----: |
| Nvidia driver |  384.130  | 384.130 |
|     CUDA      |  8.0.61   | 8.0.61  |
|     cuDNN     |  6.0.21   | 6.0.21  |
|   TensorRT    |    N/A    |   N/A   |
|    OpenCV     | 3.2.0-dev |  2.4.8  |
|    OpenMP     |    N/A    |   N/A   |

We are working on a fix to line up the OpenCV versions between the two.