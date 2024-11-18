# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import random
import qt.main_window
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, Imu
from tf_transformations import euler_from_quaternion
from qt.main_window import MainWindow
from vehicles.vehicle import Vehicle



class GroundControlStation(Node):
    def __init__(self, app, main_window):
        super().__init__('ground_control_station')
        self.gps_subscriber_ = self.create_subscription(NavSatFix, 'gps', self.gps_callback, 10)
        self.imu_subscriber_ = self.create_subscription(Imu, 'imu', self.imu_callback, 10)

        self.publisher_ = self.create_publisher(NavSatFix, 'gps', 10)
        
        self.app_ = app
        self.main_window_ = main_window
        
        self.main_window_timer = self.create_timer(0, self.main_window_timer_callback)
        self.timer_ = self.create_timer(0.5, self.timer_callback)

        self.vehicle_ = Vehicle(self, 'vehicle1')
        
    

    def gps_callback(self, msg):
        self.main_window_.update_marker(msg.latitude, msg.longitude, 0.0)
        self.get_logger().info('Received GPS: %f, %f' % (msg.latitude, msg.longitude))



    def imu_callback(self, msg: Imu):
        self.main_window_.update_marker(0.0, 0.0, msg.orientation.z)
        self.get_logger().info('Received IMU: %f' % msg.orientation.z)


    
    def timer_callback(self):
        msg = NavSatFix()
        msg.latitude = random.uniform(40.0, 40.2)
        msg.longitude = random.uniform(29.0, 29.2)
        self.publisher_.publish(msg)


    
    def main_window_timer_callback(self):
        self.app_.processEvents()


def main(args=None):
    rclpy.init(args=args)

    app = qt.main_window.QApplication([])
    main_window = MainWindow()
    main_window.show()

    ground_control_station = GroundControlStation(app, main_window)
    rclpy.spin(ground_control_station)


if __name__ == '__main__':
    main()

    