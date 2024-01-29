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
   * Format: NMEA
5. Using the IP of the Reach RTK and any free port launch this package
6. ```ros2 run reach_ros_node nmea_tcp_driver --ros-args -p host:="192.168.2.15" -p port:="9001" ```


## Driver Details

* Publishes GPS fix, velocity, and time reference
  * `/tcpfix` - NavSatFixed
  * `/tcpvel` - TwistedStamped
  * `/tcptime` - TimeReference
* Can specify the following launch parameters 
  * `~host` - IP adress of the Reach RTK GNSS (default: 'reach.local')
  * `~port` - The port number for the TCP connection (default: 123456)
  * `~frame_timeref` - Frame of the time reference (default: 'gps')
  * `~frame_gps` - Frame of the fix and velocity (default: 'gps')
  * `~use_rostime` - If set to true, ROS time is used instead of the GPS time (default: True)
  * `~use_rmc` - Use compressed RMC message (note: this does not have the covariance for the fix)

## Credit

Original starting point of the driver was the ROS driver [nmea_navsat_driver](https://github.com/ros-drivers/nmea_navsat_driver) which was then expanded by [CearLab](https://github.com/CearLab/nmea_tcp_driver) to work with the Reach RTK. Migrated to ROS2 by [Bart Boogmans](https://github.com/bartboogmans/reach_ros_node).
This package is more complete, and aims to allow for use of the Reach RTK in actual robotic systems, please open a issue if you run into any issues.
Be sure to checkout this other driver by [enwaytech](https://github.com/enwaytech/reach_rs_ros_driver) for the Reach RS.






