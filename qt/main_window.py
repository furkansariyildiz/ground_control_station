import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QPushButton, QLabel, QTextEdit, QHBoxLayout
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import *
import random



class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Hello World")
        self.setGeometry(100, 100, 800, 600)

        self.central_widget_ = QWidget()
        self.setCentralWidget(self.central_widget_)

        self.layout_ = QVBoxLayout()
        self.central_widget_.setLayout(self.layout_)

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

        self.timer_ = QTimer()
        self.timer_.timeout.connect(self.update_telemetry)
        self.timer_.start(1000)



    def send_command(self, command):
        self.message_box_.append(f"Sending Command: {command}")
        self.status_label_.setText("Status: Command Sent")



    def update_telemetry(self):
        gps = f"{random.uniform(40.0, 41.0):.6f}, {random.uniform(29.0, 30.0):.6f}"
        speed = random.uniform(0, 100)
        self.telemetry_label_.setText(f"Telemetry Information: GPS: {gps} | Speed: {speed} m/s")
        self.message_box_.append(f"Received message from vehicle: GPS: {gps} | Speed: {speed} m/s")



    def save_log(self):
        with open("gcs_log.txt", "w") as f:
            f.write(self.message_box_.toPlainText())
        self.status_label_.setText("Status: Log Saved")
        