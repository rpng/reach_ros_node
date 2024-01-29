# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Patrick Geneva
# Copyright (c) 2013, Eric Perko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import math
from builtin_interfaces.msg import Time
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from reach_ros_node.checksum_utils import check_nmea_checksum
import reach_ros_node.parser

class RosNMEADriver(object):
    def __init__(self,parent_node:Node):
        # Set parent node 
        self.parent = parent_node

        # Our publishers
        self.parent.fix_pub = self.parent.create_publisher(NavSatFix,'tcpfix',10)
        self.parent.vel_pub = self.parent.create_publisher(TwistStamped,'tcpvel',10)
        self.parent.timeref_pub = self.parent.create_publisher(TimeReference,'tcptime',10)

        # Frame of references we should publish in
        self.frame_timeref = self.parent.get_parameter('frame_timeref').value
        self.frame_gps = self.parent.get_parameter('frame_gps').value
        self.use_rostime = self.parent.get_parameter('use_rostime').value
        self.use_rmc = self.parent.get_parameter('use_rmc').value
        
        # Flags for what information we have
        self.has_fix = False
        self.has_std = False
        self.has_vel = False
        self.has_timeref = False
        # Blank messages we create
        self.msg_fix = NavSatFix()
        self.msg_vel = TwistStamped()
        self.msg_timeref = TimeReference()

    # Will process the nmea_string, and try to update our current state
    # Should try to publish as many messages as possible with the given data
    def process_line(self, nmea_string):
        
        # Check if valid message
        if not check_nmea_checksum(nmea_string):
            self.parent.get_logger().warning("Received a sentence with an invalid checksum. Sentence was: %s" % repr(nmea_string))
            return
        
        # Else we are good, lets try to process this message
        parsed_sentence = reach_ros_node.parser.parse_nmea_sentence(nmea_string)
        if not parsed_sentence:
            #rospy.logwarn("Failed to parse NMEA sentence. Sentence was: %s" % nmea_string)
            self.parent.get_logger().warning("Failed to parse NMEA sentence. Sentence was: %s" % repr(nmea_string))
            return

        # We have a good message!!
        # Lets send it to the correct method to get processed
        self.parse_GGA(parsed_sentence)
        self.parse_GST(parsed_sentence)
        self.parse_VTG(parsed_sentence)
        self.parse_RMC(parsed_sentence)

        # Special care to parse the time reference
        # This can come from either the GGA or RMC
        self.parse_time(parsed_sentence)

        # Now that we are done with processing messages
        # Lets publish what we have!
        if self.has_fix and self.has_std:
            self.parent.fix_pub.publish(self.msg_fix)
            self.msg_fix = NavSatFix()
            self.has_fix = False
            self.has_std = False
        if self.has_vel:
            self.parent.vel_pub.publish(self.msg_vel)
            self.msg_vel = TwistStamped()
            self.has_vel = False
        if self.has_timeref:
            self.parent.timeref_pub.publish(self.msg_timeref)
            self.parent.msg_timeref = TimeReference()
            self.has_timeref = False

    # Parses the GGA NMEA message type
    def parse_GGA(self,datag):
        # Check if we should parse this message
        if not b'GGA' in datag:
            return
        # Return if are using RCm
        if self.use_rmc:
            return
        # Else lets set what variables we can
        data = datag[b'GGA']
        # If using ROS time, use the current timestamp
        if self.use_rostime:
            self.msg_fix.header.stamp = self.parent.get_clock().now().to_msg()
        else:
            self.msg_fix.header.stamp = Time()
        # Set the frame ID
        self.msg_fix.header.frame_id = self.frame_gps
        # Set what our fix status should be
        gps_qual = data['fix_type']
        if gps_qual == 0:
            self.msg_fix.status.status = NavSatStatus.STATUS_NO_FIX
        elif gps_qual == 1:
            self.msg_fix.status.status = NavSatStatus.STATUS_FIX
        elif gps_qual == 2:
            self.msg_fix.status.status = NavSatStatus.STATUS_SBAS_FIX
        elif gps_qual in (4, 5):
            self.msg_fix.status.status = NavSatStatus.STATUS_GBAS_FIX
        else:
            self.msg_fix.status.status = NavSatStatus.STATUS_NO_FIX
        self.msg_fix.status.service = NavSatStatus.SERVICE_GPS
        # Set our lat lon position
        latitude = data['latitude']
        if data['latitude_direction'] == 'S':
            latitude = -latitude
        self.msg_fix.latitude = latitude
        longitude = data['longitude']
        if data['longitude_direction'] == 'W':
            longitude = -longitude
        self.msg_fix.longitude = longitude
        # Altitude is above ellipsoid, so adjust for mean-sea-level
        self.msg_fix.altitude = data['altitude'] + data['mean_sea_level']
        self.has_fix = True




    # Parses the GST NMEA message type
    def parse_GST(self,datag):
        # Check if we should parse this message
        if not b'GST' in datag:
            return
        # Return if are using RCM
        if self.use_rmc:
            return
        # Else lets set what variables we can
        data = datag[b'GST']
        # Update the fix message with std
        self.msg_fix.position_covariance[0] = pow(data['latitude_sigma'],2)
        self.msg_fix.position_covariance[4] = pow(data['longitude_sigma'],2)
        self.msg_fix.position_covariance[8] = pow(data['altitude_sigma'],2)
        self.msg_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.has_std = True


    # Parses the VTG NMEA message type
    def parse_VTG(self,datag):
        # Check if we should parse this message
        if not b'VTG' in datag:
            return
        # Return if are using RCM
        if self.use_rmc:
            return
        # Else lets set what variables we can
        data = datag[b'VTG']
        # Next lets publish the velocity we have
        # If using ROS time, use the current timestamp
        # Else this message doesn't provide a time, so just set to zero
        if self.use_rostime:
            self.msg_vel.header.stamp = self.parent.get_clock().now().to_msg()
        else:
            self.msg_vel.header.stamp = Time()
        # Set the frame ID
        self.msg_vel.header.frame_id = self.frame_gps
        # Calculate the change in orientatoin
        self.msg_vel.twist.linear.x = data['speed'] * math.sin(data['ori_true'])
        self.msg_vel.twist.linear.y = data['speed'] * math.cos(data['ori_true'])
        self.has_vel = True




    # Parses the RMC NMEA message type
    def parse_RMC(self,datag):
        # Check if we should parse this message
        if not b'RMC' in datag:
            return
        # Return if not using RCM
        if not self.use_rmc:
            return
        # Else lets set what variables we can
        data = datag[b'RMC']
        # If using ROS time, use the current timestamp
        if self.use_rostime:
            self.msg_fix.header.stamp = self.parent.get_clock().now().to_msg()
        else:
            self.msg_vel.header.stamp = Time()
        # Set the frame ID
        self.msg_fix.header.frame_id = self.frame_gps
        # Update the fix message
        if data['fix_valid']:
            self.msg_fix.status.status = NavSatStatus.STATUS_FIX
        else:
            self.msg_fix.status.status = NavSatStatus.STATUS_NO_FIX
        self.msg_fix.status.service = NavSatStatus.SERVICE_GPS
        # Set our lat lon position
        latitude = data['latitude']
        if data['latitude_direction'] == 'S':
            latitude = -latitude
        self.msg_fix.latitude = latitude
        longitude = data['longitude']
        if data['longitude_direction'] == 'W':
            longitude = -longitude
        self.msg_fix.longitude = longitude
        # When using RMC we don't know the height and covariance
        self.msg_fix.altitude = float('NaN')
        self.msg_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.has_fix = True
        self.has_std = True

        # Next lets publish the velocity we have
        # If using ROS time, use the current timestamp
        if self.use_rostime:
            self.msg_vel.header.stamp = self.parent.get_clock().now().to_msg()
        else:
            self.msg_vel.header.stamp = Time()
        # Set the frame ID
        self.msg_vel.header.frame_id = self.frame_gps
        # Calculate the change in orientatoin
        self.msg_vel.twist.linear.x = data['speed'] * math.sin(data['true_course'])
        self.msg_vel.twist.linear.y = data['speed'] * math.cos(data['true_course'])
        self.has_vel = True

    # Parses the NMEA messages and just grab the time reference
    def parse_time(self,datag):

        # Get our message data
        if not self.use_rmc and b'GGA' in datag:
            data = datag[b'GGA']
        elif self.use_rmc and b'RMC' in datag:
            data = datag[b'RMC']
        else:
            return
        # Return if time is NaN
        if math.isnan(data['utc_time']):
            return
        # If using ROS time, use the current timestamp
        if self.use_rostime:
            self.msg_timeref.header.stamp = self.parent.get_clock().now().to_msg()
        else:
            self.msg_vel.header.stamp = Time()
        # Set the frame ID
        self.msg_timeref.header.frame_id = self.frame_timeref
        # Set the actuall time reference
        self.msg_timeref.time_ref = Time()
        self.msg_timeref.source = self.frame_timeref
        self.has_timeref = True