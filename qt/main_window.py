import sys
import os
import folium
import random
import io
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QPushButton, QLabel, QTextEdit, QHBoxLayout
from PyQt5.QtWebEngineWidgets import QWebEngineView, QWebEngineSettings
from PyQt5.QtCore import Qt, QTimer,QUrl
from PyQt5.QtGui import *



class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Hello World")
        self.setGeometry(100, 100, 800, 600)

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

        self.control_layout_ = QHBoxLayout()
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
        self.message_box_.setReadOnly(True)
        self.message_box_.setPlaceholderText("Vehicle messages will appear here")
        self.layout_.addWidget(self.message_box_)

        self.save_log_button_ = QPushButton("Save Log")
        self.layout_.addWidget(self.save_log_button_)

        self.forward_button_.clicked.connect(lambda: self.send_command("forward"))
        self.backword_button_.clicked.connect(lambda: self.send_command("backward"))
        self.left_button_.clicked.connect(lambda: self.send_command("left"))
        self.right_button_.clicked.connect(lambda: self.send_command("right"))
        self.save_log_button_.clicked.connect(self.save_log)
        
        # Set the URL of the map
        self.map_view_.setUrl(QUrl.fromLocalFile(os.path.abspath("src/ground_control_station/html/folium/dynamic_map.html")))
        
        # Set the path of the vehicle icon
        self.vehicle_icon_path_ = str(os.path.abspath("src/ground_control_station/html/icons/arrow.png"))
        self.map_view_.loadFinished.connect(self.set_icon)



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



    def update_telemetry(self):
        gps_latitude = random.uniform(40.0, 40.0)
        gps_longitude = random.uniform(29.0, 30.0)
        speed = random.uniform(0, 100)
        self.telemetry_label_.setText(f"Telemetry Information: {gps_latitude:.6f}, {gps_longitude:.6f} | Speed: {speed} m/s")
        self.message_box_.append(f"Received Message from Vehicle: GPS={gps_latitude:.6f}, {gps_longitude:.6f}, Speed={speed}")

    

    def update_marker(self, latitude, longitude, angle):
        js_command = f"updateMarker({latitude}, {longitude}, {angle});"
        self.map_view_.page().runJavaScript(js_command)
        


    def update_map_file(self):
        file_path = os.path.abspath("map.html")  # Tam dosya yolu oluştur
        self.map_.save(file_path)



    def save_log(self):
        with open("gcs_log.txt", "w") as f:
            f.write(self.message_box_.toPlainText())
        self.status_label_.setText("Status: Log Saved")
        


if __name__ == "__main__":
    app = QApplication([])
    window = MainWindow()
    window.show()
    app.exec()