# Reach RTK ROS Node


This is a very simple ROS node that allows for publishing of NMEA messages onto the ROS framework.
This package aims to support the [Reach RTK GNSS](https://emlid.com/shop/reach-rtk-kit/) module by Emlid.
Right now this supports all NMEA messages from the package, while some are not used to publish anything onto ROS.


## Quickstart Guide

1. Clone this package into your ROS workspace and build it
2. Turn on your Reach RTK module
3. Ensure that you are connected to the same network as it
4. Edit the "Position output"
   * Be in TCP mode
   * Role: Server
   * Address: localhost
   * Port: any free port
   * Format: NMEA
5. Using the IP of the Reach RTK and the specified port launch this package
6. `rosrun reach_ros_node nmea_tcp_driver _host:=128.4.89.123 _port:=2234`


## Driver Details

* Publishes GPS fix, velocity, and time reference
  * `/tcpfix` - NavSatFixed
  * `/tcpvel` - TwistedStamped
  * `/tcptime` - TimeReference
* Can specify the following launch parameters
  * `~frame_timeref` - Frame of the time reference
  * `~frame_gps` - Frame of the fix and velocity
  * `~use_rostime` - If set to true, ROS time is used instead of the GPS time
  * `~use_rmc` - Use compressed RMC message (note: this does not have the covariance for the fix)



## Credit

Original starting point of the driver was the ROS driver [nmea_navsat_driver](https://github.com/ros-drivers/nmea_navsat_driver) which was then expanded by [CearLab](https://github.com/CearLab/nmea_tcp_driver) to work with the Reach RTK.
This package is more complete, and aims to allow for use of the Reach RTK in actual robotic systems, please open a issue if you run into any issues.
Be sure to checkout this other driver by [enwaytech](https://github.com/enwaytech/reach_rs_ros_driver) for the Reach RS.






