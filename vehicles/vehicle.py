import rclpy
import random
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from tf_transformations import euler_from_quaternion



class Vehicle:
    def __init__(self, node, vehicle_id):
        self.__node = node
        self.vehicle_id = vehicle_id

        self.gps_subscriber_ = self.__node.create_subscription(NavSatFix, vehicle_id + '/gps', self.gps_callback, 10)
        self.imu_subscriber_ = self.__node.create_subscription(Imu, vehicle_id + '/imu', self.imu_callback, 10)

        self.gps_publisher_ = self.__node.create_publisher(NavSatFix, vehicle_id + '/gps', 10)
        self.imu_publisher_ = self.__node.create_publisher(Imu, vehicle_id + '/imu', 10)

        self.__latitude = 0.0
        self.__longitude = 0.0
        self.__altitude = 0.0

        self.__roll = 0.0
        self.__pitch = 0.0
        self.__yaw = 0.0

        self.timer_ = self.__node.create_timer(0.1, self.timer_callback)
        



    def gps_callback(self, msg):
        self.__latitude = msg.latitude
        self.__longitude = msg.longitude
        self.__altitude = msg.altitude
        self.__node.get_logger().info('Received GPS: %f, %f' % (msg.latitude, msg.longitude))


    
    def imu_callback(self, msg: Imu):
        (self.__roll, self.__pitch, self.__yaw) = euler_from_quaternion([msg.orientation.x, 
                                                     msg.orientation.y, 
                                                     msg.orientation.z, 
                                                     msg.orientation.w])
        
        self.__node.get_logger().info('Roll: %f, Pitch: %f, Yaw: %f' % (self.__roll, self.__pitch, self.__yaw))
    


    def timer_callback(self):
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



    def get_latitude(self):
        return self.__latitude
    


    def get_longitude(self):
        return self.__longitude
    


    def get_altitude(self):
        return self.__altitude
    


    def get_roll(self):
        return self.__roll
    


    def get_pitch(self):
        return self.__pitch
    


    def get_yaw(self):
        return self.__yaw