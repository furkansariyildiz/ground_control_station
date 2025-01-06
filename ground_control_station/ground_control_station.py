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
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication
from rclpy.node import Node
from qt.main_window import MainWindow
import signal
import sys



class GroundControlStation(Node):
    def __init__(self, app):
        super().__init__('ground_control_station')

        # Start Qt Main Window
        self.main_window_ = MainWindow(self)
        self.main_window_.show()

        self.app_ = app
        self.get_logger().info('Ground Control Station Node has been initialized')

        # Qt and ROS event loop
        self.main_window_timer_ = QTimer()
        self.main_window_timer_.timeout.connect(self.timer_callback)
        self.main_window_timer_.start(10)  

    def timer_callback(self):
        self.app_.processEvents()  # Qt olaylarını işler
        rclpy.spin_once(self, timeout_sec=0)  # ROS olaylarını işler

    def cleanup(self):
        self.get_logger().info('Cleaning up resources...')
        self.main_window_timer_.stop()
        self.main_window_.close()


def signal_handler(ground_control_station):
    ground_control_station.cleanup()  
    rclpy.shutdown()  
    sys.exit(0)  


def main(args=None):
    rclpy.init(args=args)
    app = QApplication([])

    ground_control_station = GroundControlStation(app)

    signal.signal(signal.SIGINT, lambda sig, frame: signal_handler(ground_control_station))
    signal.signal(signal.SIGTERM, lambda sig, frame: signal_handler(ground_control_station))

    try:
        app.exec()  
    except Exception as e:
        print(f"Exception: {e}")
    finally:
        ground_control_station.cleanup()  
        rclpy.shutdown()


if __name__ == '__main__':
    main()

