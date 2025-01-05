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
from collections import defaultdict



class GroundControlStation(Node):
    def __init__(self, app):
        super().__init__('ground_control_station')
        
        self.main_window_ = MainWindow(self)
        self.main_window_.show()

        self.app_ = app
        self.get_logger().info('Ground Control Station Node has been initialized')
        self.main_window_timer = self.create_timer(0, self.main_window_timer_callback)



    def main_window_timer_callback(self):
        self.app_.processEvents()



def main(args=None):
    rclpy.init(args=args)
    app = qt.main_window.QApplication([])
    ground_control_station = GroundControlStation(app)
    rclpy.spin(ground_control_station)



if __name__ == '__main__':
    main()

    