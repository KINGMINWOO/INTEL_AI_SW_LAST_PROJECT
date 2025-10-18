#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import RPi.GPIO as GPIO
import time

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('line_sensor_publisher')
        self.pub = self.create_publisher(Int32MultiArray, '/line_sensor', 10)

        GPIO.setmode(GPIO.BCM)
        self.left = 17
        self.right = 27
        GPIO.setup(self.left, GPIO.IN)
        GPIO.setup(self.right, GPIO.IN)

        self.timer = self.create_timer(0.1, self.publish_sensor)
        self.get_logger().info("📡 Line Sensor Publisher Started!")

    def publish_sensor(self):
        L = GPIO.input(self.left)
        R = GPIO.input(self.right)
        msg = Int32MultiArray()
        msg.data = [L, R]
        self.pub.publish(msg)
        self.get_logger().info(f"Sensor → L:{L}, R:{R}")

def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
