import rclpy
from rclpy.node import Node

ros_initialized = False

def ros_init():
    global ros_initialized
    if not rclpy.ok():
        rclpy.init()
    if not ros_initialized:
        ros_initialized = True
