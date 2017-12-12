[![Waffle.io - Columns and their card count](https://badge.waffle.io/knight-riders/CarND-Capstone-Project.png?columns=all)](https://waffle.io/knight-riders/CarND-Capstone-Project?utm_source=badge)

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

### Team Members
|Name            |Udacity account email|
|----------------- |-----------------------------|
|Mario Landry (**Team Lead**) | mario.landry AT gmail.com |
|Rob Fitch | rob.m.fitch AT gmail.com |
|Rafael Rivas | rafael.rivas.dev AT gmail.com |
|Patrick Poon | patrickmpoon AT gmail.com |

### Project Implementation

## Waypoint Updater
The waypoint updater is responsible for calculating the next LOOKAHEAD_WPS (200) Waypoints and setting a target velocity for each. The default target velocity at each waypoint is set at system startup and recorded into the base waypoints. This velocity is used as a ceiling for the vehicle velocity and intentionally left unaltered by the waypoint updater. At each call to the waypoint updater, a subset of the base waypoint list is created of length LOOKAHEAD_WPS, starting with the calculated NEXT_WAYPOINT. The velocities of these waypoints are then set to allow smooth and safe deceleration for any obstacles the car encounters. To aid in this, the current vehicle velocity and maximum acceleration value are used to determine the distance required by the vehicle to safely stop. This value is recorded as the deceleration zone or decel_zone.  

The waypoint updater subscribes to a list of obstacles as well as a waypoint for the next light. If either of these are populated, they indicate zones the car should not pass through. The traffic light waypoint, if present, is appended to the end of the list of obstacles. Each waypoint in the obstacle list is accompanied by a standoff distance, indicating how far before reaching the obstacle the car should attempt to stop. The standoff value is determined based on the subscription providing the obstacle.  

For every waypoint in the obstacle list found to be within the respective deceleration zone plus standoff distance, a stopping gradient is calculated. The velocity at each final waypoint is the lowest value among these stopping gradients and the velocity already set for the waypoint.  

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity team provided a virtual machine that has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

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
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car (a bag demonstraing the correct predictions in autonomous mode can be found [here](https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc))
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
