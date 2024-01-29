#! /usr/bin/env python

# Software License Agreement (BSD License)
#
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

import socket
import rclpy
import reach_ros_node.driver
from rclpy.node import Node

class ros2_ReachSocketHandler(Node):
    # Set our parameters and the default socket to open
    def __init__(self):
        super().__init__('reach_ros_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('host', 'reach.local'),
                ('port', 12346),
                ('frame_gps', 'gps'),
                ('frame_timeref', 'gps'),
                ('use_rostime', True),
                ('use_rmc', False)]
        )
             
    # Should open the connection and connect to the device
    # This will then also start publishing the information
    def start(self):
        # Try to connect to the device
        self.get_logger().info('Connecting to Reach RTK %s on port %s' % (str(self.get_parameter('host').value),str(self.get_parameter('port').value)))
        self.connect_to_device()
        try:
            driver = reach_ros_node.driver.RosNMEADriver(self)
        except Exception as e:
            self.get_logger().error("an error occured while trying to make the driver. Error was: %s." %e)

        try:
            while rclpy.ok():
                data = self.buffered_readLine().strip()  
                rclpy.spin_once(self, timeout_sec=0.1) # allow functions such as ros2 node info, ros2 param list to work
                try:
                    driver.process_line(data) 
                except ValueError as e:
                    self.get_logger().info("Value error, likely due to missing fields in the NMEA message. Error was: %s." % e)
        except Exception as e:
            self.get_logger().error("an error occured while reading lines from device. Error was: %s." %e)
            # Close GPS socket when done
            self.soc.close()

    # Try to connect to the device, allows for reconnection
    # Will loop till we get a connection, note we have a long timeout
    def connect_to_device(self):
        while rclpy.ok():
            try:
                self.soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.soc.settimeout(5.0)
                self.soc.connect((self.get_parameter('host').value,self.get_parameter('port').value))
                self.get_logger().info('Successfully connected to device, starting publishing!')
                return
            except socket.timeout:
                self.get_logger().warning(30,'Socket connection timeout. Retrying...')
                continue
            except Exception as e:
                self.get_logger().error("Socket connection error. Error was: %s." % e)
                exit()

    # Try to connect to the device, assuming it just was disconnected
    # Will loop till we get a connection
    def reconnect_to_device(self):
        self.get_logger().warning('Device disconnected. Reconnecting...')
        self.soc.close()
        self.connect_to_device()

    # Read one line from the socket
    # We want to read a single line as this is a single nmea message
    # https://stackoverflow.com/a/41333900
    # Also set a timeout so we can make sure we have a valid socket
    # https://stackoverflow.com/a/15175067
    def buffered_readLine(self):
        line = b""
        while rclpy.ok():
            # Try to get data from it
            try:
                part = self.soc.recv(1)
            except socket.timeout:
                self.reconnect_to_device()
                continue
            # See if we need to process the data
            if not part or len(part) == 0:
                self.reconnect_to_device()
                continue
            if part != b"\n":
                line += part
            elif part == b"\n":
                break
        return line
    
def main(args=None):
	rclpy.init(args=args)

	node = ros2_ReachSocketHandler()
	

	# Start the nodes processing thread
	node.start()
    #rclpy.spin(node)

	# at termination of the code (generally with ctrl-c) Destroy the node explicitly
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()