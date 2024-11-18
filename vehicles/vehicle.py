import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from tf_transformations import euler_from_quaternion



class Vehicle:
    def __init__(self, node, vehicle_id):
        self.__node = node

        self.gps_subscriber_ = self.__node.create_subscription(NavSatFix, vehicle_id + '/gps', self.gps_callback, 10)
        self.imu_subscriber_ = self.__node.create_subscription(Imu, vehicle_id + '/imu', self.imu_callback, 10)

        self.__latitude = 0.0
        self.__longitude = 0.0
        self.__altitude = 0.0

        self.__roll = 0.0
        self.__pitch = 0.0
        self.__yaw = 0.0



    def gps_callback(self, msg):
        self.__latitude = msg.latitude
        self.__longitude = msg.longitude
        self.__altitude = msg.altitude
        self.__node.get_logger().info('Received GPS: %f, %f' % (msg.latitude, msg.longitude))


    
    def imu_callback(self, msg: Imu):
        self.get_logger().info('Received IMU: %f' % msg.orientation.z)
        (self.__roll, self.__pitch, self.__yaw) = euler_from_quaternion([msg.orientation.x, 
                                                     msg.orientation.y, 
                                                     msg.orientation.z, 
                                                     msg.orientation.w])
        
        self.__node.get_logger().info('Roll: %f, Pitch: %f, Yaw: %f' % (self.__roll, self.__pitch, self.__yaw))
    