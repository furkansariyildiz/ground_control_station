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
import qt.main_window
from rclpy.node import Node
from std_msgs.msg import String
from qt.main_window import MainWindow

class GroundControlStation(Node):
    def __init__(self):
        super().__init__('ground_control_station')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer_ = self.create_timer(0.5, self.timer_callback)
        timer_period = 0.5


    
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, World!'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    
    app = qt.main_window.QApplication(qt.main_window.sys.argv)
    main_window = MainWindow()
    main_window.show()

    ground_control_station = GroundControlStation()

    try:
        while rclpy.ok():
            rclpy.spin_once(ground_control_station, timeout_sec=0.01)
            app.processEvents()
    except KeyboardInterrupt:
        pass
    finally:
        ground_control_station.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()

    