This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

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
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 127.0.0.1:4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
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

### Waypoint Updater

![waypoint updater block](./waypoint-updater-ros-graph.png)

The waypoint updater node takes the car's current pose and waypoint information, to compute and publish the future set of final waypoints which the car's drive by wire (dbw) system can follow.

Before the waypoint updater can start, it needs to receive the base_waypoints -- the set of points which describes the path on the road which the car should follow.  For the simulator, the base_waypoints describes a loop around the test road. The initial base_waypoints also contains a set of vehicle velocities which can be used to initially test the waypoint updater, however, for the complete waypoint updater, those values will be recomputed.  

When the waypoint updater node first receives the base_waypoints, it resets all of the velocities to a minimum since it doesn't necessarily know the current car pose and consequently closest waypoint.  It must also accelerate the vehicle from a standstill. Since the car's starting position in the simulator is relatively close to a traffic light, it could actually receive an upcoming traffic waypoint while it's still accelerating from a standstill.  If the car responds to these traffic waypoints and starts to slow down too early, it will "crawl" until it reaches the first traffic light.  On the other hand, if the waypoint updater allows the car to reach its cruise velocity (about 24mph in the simulator), it will not have enough  time to slow down.  Consequently, the waypoint updater goes through an initialiaization state where it will ignore traffic waypoints.  Once it reaches an initial velocity of about 1/5 the cruise velocity, it will then go into normal mode and respond to traffic waypoint messages.

When the waypoint updater receives the current car pose, it must first calculate the closest waypoint.  To do this efficiently, if it previously computed a closest waypoint, it will use that as a starting point and search around that; otherwise it will search the entire list of base_waypoints.  Once it computes a candidate for closest waypoint, it compares the directional vector from the car's current position to that waypoint versus the car's current orientation (yaw) in order to determine if the waypoint is in front or in back of the car.  If the waypoint is behind the car, it will increment the waypoint by one.  If by chance, the waypoint matches the current car location, it will increment it again so that the car can move forward. 

Once the waypoint updater finds the next closest waypoint to follow, it checks to see if there is an upcoming traffic waypoint.  If there is no traffic waypoint, it will take the closest waypoint's velocity and for the next LOOKAHEAD_WPS number of waypoints calculate the appropriate velocities such that it either accelerates to the cruise velocity or stays at the cruise velocity.  If there is an upcoming traffic waypoint, however, it will calculate how far it is from its stopping point and how long it would take to stop using maximum deceleration.  If there is sufficient margin, it will continue to cruise at its current velocity; otherwise it will calculate a trajectory where it slows down.

For both accelerating and decelerating, the waypoint updater uses the distance between two waypoints and the simple kinematic equation: *Vf <sup>2</sup>* = *Vi <sup>2</sup>* + *2&middot;a&middot;s*, where :

* *Vf* : next waypoint velocity
* *Vi* : current waypoint velocity
* *a*  : acceleration or deceleration where acceleration is > 0 and deceleration < 0
* *s*  : distance between waypoints

When the waypoint updater receives a traffic waypoint, it also performs some checks to ensure that it is valid.  If the traffic waypoint is behind the car or outside of it's projected trajectory, then it will ignore it.  If the traffic waypoint is -1 and it's stopped at a traffic waypoint, then it clears its current traffic waypoint and can start accelerating to its target cruise velocity once again.

In the simulator, the base_waypoints forms a loop around the test road.  To support looping around the track, modular arithmetic is used when dealing with waypoint indices.  For example if there are NUM_WAYPOINTS which are indexed from 0 to NUM_WAYPOINTS-1, NUM_WAYPOINTS and NUM_WAYPOINTS+1 will map back to index 0 and 1 respectively.


