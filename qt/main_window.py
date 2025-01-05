import sys
import os
import folium
import random
import io
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QPushButton, QLabel, QTextEdit, QHBoxLayout, QListWidget, QComboBox
from PyQt5.QtWebEngineWidgets import QWebEngineView, QWebEngineSettings
from PyQt5.QtCore import Qt, QTimer,QUrl
from PyQt5.QtGui import *
from vehicles.vehicle import Vehicle
from std_msgs.msg import String



class MainWindow(QMainWindow):
    def __init__(self, node: Node):
        super().__init__()
        # Getting ROS2 node
        self.__node = node

        # Subscribers

        # Publishers
        self.selected_vehicle_publisher = self.__node.create_publisher(String, '/selected_vehicle', 10)

        # ROS2 Paramters (Declare and Get)
        self.__node.declare_parameters(
            namespace='',
            parameters=[
                ('vehicle_list.vehicle1.vehicle_id', rclpy.Parameter.Type.INTEGER),
                ('vehicle_list.vehicle1.vehicle_type', rclpy.Parameter.Type.STRING),
                ('vehicle_list.vehicle1.vehicle_description', rclpy.Parameter.Type.STRING),
                ('vehicle_list.vehicle_2.vehicle_id', rclpy.Parameter.Type.INTEGER),
                ('vehicle_list.vehicle_2.vehicle_type', rclpy.Parameter.Type.STRING),
                ('vehicle_list.vehicle_2.vehicle_description', rclpy.Parameter.Type.STRING),
            ]
        )

        vehicle_params = self.__node.get_parameters_by_prefix('vehicle_list')
        
        vehicles = {}  

        for full_key, parameter_obj in vehicle_params.items():
            parts = full_key.split('.')
            if len(parts) == 2:  
                vehicle_name, param_name = parts
                if vehicle_name not in vehicles:
                    vehicles[vehicle_name] = {}
                vehicles[vehicle_name][param_name] = parameter_obj.value

        for vehicle_name, params in vehicles.items():
            self.__node.get_logger().info(
                f"Vehicle: {vehicle_name}, ID: {params.get('vehicle_id')}, "
                f"Type: {params.get('vehicle_type')}, Description: {params.get('vehicle_description')}"
            )

        # Create Vehicle objects
        vehicle_1 = Vehicle(self.__node, 'vehicle1')
        vehicle_2 = Vehicle(self.__node, 'vehicle2')
        self.vehicle_objects_ = [vehicle_1, vehicle_2]

        self.setWindowTitle("Ground Control Station")
        self.setGeometry(100, 100, 800, 600)

        # Layouts
        self.control_layout_ = QHBoxLayout()
        self.vehicle_select_layout_ = QHBoxLayout()

        self.central_widget_ = QWidget()
        self.setCentralWidget(self.central_widget_)

        self.layout_ = QVBoxLayout()
        self.central_widget_.setLayout(self.layout_)

        # Create a map
        self.map_view_ = QWebEngineView()
        self.layout_.addWidget(self.map_view_)

        self.load_map(40.0, 30.0)

        self.status_label_ = QLabel("Status: Not Connected")
        self.layout_.addWidget(self.status_label_)

        self.telemetry_label_ = QLabel("Telemetry Information: No GPS Data | Speed: 0 m/s")
        self.layout_.addWidget(self.telemetry_label_)

        self.forward_button_ = QPushButton("Forward")
        self.backword_button_ = QPushButton("Backward")
        self.left_button_ = QPushButton("Left")
        self.right_button_ = QPushButton("Right")   
        self.control_layout_.addWidget(self.forward_button_)
        self.control_layout_.addWidget(self.backword_button_)
        self.control_layout_.addWidget(self.left_button_)
        self.control_layout_.addWidget(self.right_button_)
        self.layout_.addLayout(self.control_layout_)

        self.message_box_ = QTextEdit()
        self.message_box_.setFixedSize(700, 400)  # Set the desired width and height
        self.message_box_.setReadOnly(True)
        self.message_box_.setPlaceholderText("Vehicle messages will appear here")
        self.layout_.addWidget(self.message_box_)

        self.save_log_button_ = QPushButton("Save Log")
        self.layout_.addWidget(self.save_log_button_)
        
        # Create a combo box for vehicle selection 
        self.combo_box_ = QComboBox()        
        for vehicle in self.vehicle_objects_:
            self.combo_box_.addItem(vehicle.vehicle_id)

        # Add a button to get the selected option
        self.selected_vehicle_id = QPushButton("Selected Vehicle ID")
        self.vehicle_select_layout_.addWidget(self.selected_vehicle_id)
        self.vehicle_select_layout_.addWidget(self.combo_box_)

        # Add vehicle_select_layout to the main layout at the desired position
        self.layout_.insertLayout(0, self.vehicle_select_layout_)  # Add at the top

        self.forward_button_.clicked.connect(lambda: self.send_command("forward"))
        self.backword_button_.clicked.connect(lambda: self.send_command("backward"))
        self.left_button_.clicked.connect(lambda: self.send_command("left"))
        self.right_button_.clicked.connect(lambda: self.send_command("right"))
        self.save_log_button_.clicked.connect(self.save_log)
        
        # Set the URL of the map
        self.map_view_.setUrl(QUrl.fromLocalFile(os.path.abspath("src/ground_control_station/html/folium/dynamic_map.html")))
        
        # Set the path of the vehicle icon
        self.vehicle_icon_path_ = str(os.path.abspath("src/ground_control_station/html/icons/arrow.png"))



    def update_vehicle_status_timer_callback(self):
        self.update_marker(self.vehicle_1_.vehicle_id,
                            self.vehicle_1_.get_latitude(), 
                            self.vehicle_1_.get_longitude(), 
                            self.vehicle_1_.get_yaw())
        
        self.update_marker(self.vehicle_2_.vehicle_id,
                            self.vehicle_2_.get_latitude(), 
                            self.vehicle_2_.get_longitude(), 
                            self.vehicle_2_.get_yaw())
        
        self.update_telemetry(self.vehicle_1_.vehicle_id,
                              self.vehicle_1_.get_latitude(), 
                              self.vehicle_1_.get_longitude(), 
                              self.vehicle_1_.get_speed())
        
        self.update_telemetry(self.vehicle_2_.vehicle_id,
                              self.vehicle_2_.get_latitude(), 
                              self.vehicle_2_.get_longitude(), 
                              self.vehicle_2_.get_speed())



    def set_icon(self):
        js_command = f"setVehicleIcon('{self.vehicle_icon_path_}');"
        self.map_view_.page().runJavaScript(js_command)



    def send_command(self, command):
        self.message_box_.append(f"Sending Command: {command}")
        self.status_label_.setText("Status: Command Sent")



    def load_map(self, latitude, longitude):
        m = folium.Map(location=[latitude, longitude], zoom_start=14)
        folium.Marker([latitude, longitude], popup="Vehicle").add_to(m)
        
        data = io.BytesIO()
        m.save(data, close_file=False)
        self.map_view_.setHtml(data.getvalue().decode())



    def update_telemetry(self, vehicle_name: str, gps_latitude: float, gps_longitude: float, speed: float):
        self.telemetry_label_.setText(f"Telemetry Information: {gps_latitude:.6f}, {gps_longitude:.6f} | Speed: {speed} m/s")
        self.message_box_.append(f"Received Message from Vehicle: GPS={gps_latitude:.6f}, {gps_longitude:.6f}, Speed={speed}")

    

    def update_marker(self, vehicle_name, latitude, longitude, angle):
        js_command = f"updateMarker('{vehicle_name}', {latitude}, {longitude}, {angle}, '{self.vehicle_icon_path_}');"
        self.map_view_.page().runJavaScript(js_command)
        


    def update_map_file(self):
        file_path = os.path.abspath("map.html") 
        self.map_.save(file_path)



    def save_log(self):
        with open("gcs_log.txt", "w") as f:
            f.write(self.message_box_.toPlainText())
        self.status_label_.setText("Status: Log Saved")
        

    def get_selected_vehicle(self):
        selected_vehicle = self.combo_box_.currentText()
        self.message_box_.append(f"Selected Option: {selected_vehicle}")


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node('ground_control_station')
    app = QApplication([])
    window = MainWindow(node)
    window.show()
    rclpy.spin(node)
    rclpy.shutdown()
    app.exec()