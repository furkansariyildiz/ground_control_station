import sys
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow
from PyQt5.QtCore import Qt
from PyQt5.QtGui import *


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Hello World")
        self.setGeometry(100, 100, 800, 600)

        self.label_ = QLabel("Hello World")
        self.label_.setGeometry(200, 200, 400, 100)
        self.label_.setAlignment(Qt.AlignmentFlag.AlignCenter)

        