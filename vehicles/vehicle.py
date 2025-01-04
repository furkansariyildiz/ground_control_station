import rclpy
import random
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion



class Vehicle:
    def __init__(self, node, vehicle_id):
        self.__node = node
        self.vehicle_id = vehicle_id

        self.gps_subscriber_ = self.__node.create_subscription(NavSatFix, vehicle_id + '/gps', self.gps_callback, 10)
        self.imu_subscriber_ = self.__node.create_subscription(Imu, vehicle_id + '/imu', self.imu_callback, 10)
        self.cmd_vel_subscriber_ = self.__node.create_subscription(Twist, vehicle_id + '/cmd_vel', self.cmd_vel_callback, 10)

        self.gps_publisher_ = self.__node.create_publisher(NavSatFix, vehicle_id + '/gps', 10)
        self.imu_publisher_ = self.__node.create_publisher(Imu, vehicle_id + '/imu', 10)
        self.cmd_vel_publisher_ = self.__node.create_publisher(Twist, vehicle_id + '/cmd_vel', 10)

        self.__latitude = 0.0
        self.__longitude = 0.0
        self.__altitude = 0.0

        self.__roll = 0.0
        self.__pitch = 0.0
        self.__yaw = 0.0

        self.__twist_message = Twist()

        self.timer_ = self.__node.create_timer(0.1, self.timer_callback)
        



    def gps_callback(self, msg):
        """
        GPS callback function.

        ***Args**:
            - msg (NavSatFix): GPS message
        """
        self.__latitude = msg.latitude
        self.__longitude = msg.longitude
        self.__altitude = msg.altitude


    
    def imu_callback(self, msg: Imu):
        """
        IMU callback function.

        ***Args**:
            - msg (Imu): IMU message
        """
        (self.__roll, self.__pitch, self.__yaw) = euler_from_quaternion([msg.orientation.x, 
                                                     msg.orientation.y, 
                                                     msg.orientation.z, 
                                                     msg.orientation.w])
        


    def cmd_vel_callback(self, msg: Twist):
        """
        Command velocity callback function.

        ***Args**:
            - msg (Twist): Twist message
        """
        self.__twist_message = msg
        
    

    def timer_callback(self):
        """
        Publisher random GPS and IMU messages for simulation purposes

        ***Args**:
            - None
        """
        gps_msg = NavSatFix()
        gps_msg.latitude = random.uniform(40.0, 40.05)
        gps_msg.longitude = random.uniform(29.0, 29.05)
        gps_msg.altitude = self.__altitude
        self.gps_publisher_.publish(gps_msg)

        imu_msg = Imu()
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0
        self.imu_publisher_.publish(imu_msg)

        twist_msg = Twist()
        twist_msg.linear.x = random.uniform(0.0, 1.0)
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher_.publish(twist_msg)



    def get_latitude(self):
        """
        Getter function for latitude

        ***Args**:
            - None
        """
        return self.__latitude
    


    def get_longitude(self):
        """
        Getter function for longitude

        ***Args**:
            - None
        """
        return self.__longitude
    


    def get_altitude(self):
        """
        Getter function for altitude

        ***Args**:
            - None
        """
        return self.__altitude
    


    def get_roll(self):
        """
        Getter function for roll

        ***Args**:
            - None
        """
        return self.__roll
    


    def get_pitch(self):
        """
        Getter function for pitch

        ***Args**:
            - None
        """
        return self.__pitch
    


    def get_yaw(self):
        """
        Getter function for yaw

        ***Args**:
            - None
        """
        return self.__yaw
    


    def get_speed(self):
        """
        Getter function for speed

        ***Args**:
            - None
        """
        return self.__twist_message.linear.x