
from sensor_msgs.msg import Range
from std_msgs.msg import Bool

from rclpy.node import Node
import rclpy
import serial

class SensorReader(Node):
    def __init__(self, port, baud):
        super().__init__('ultrasonic_reader')

        self.speed_sound = 340.0 # m/s

        self.left_pub = self.create_publisher(Range, 'range_left', 10)
        self.right_pub = self.create_publisher(Range, 'range_right', 10)
        self.reset_pub = self.create_publisher(Bool, 'reset', 10)

        self.serial_port = serial.Serial(port, baud)
        print(f'Connected to Arduino on {port} at {baud} baud')

    def close(self):
        self.serial_port.close()


    def run(self):
        while rclpy.ok():
            if self.serial_port.in_waiting:
                try:
                    data = self.serial_port.readline().decode('utf-8').strip()
                except:
                    continue
                print(data)
                try:
                    stamp_ms, left_pulse_time, right_pulse_time, button_pressed = data.split(',')
                except:
                    continue
                left_distance = int(left_pulse_time) * 1e-6 * self.speed_sound / 2.0
                right_distance = int(right_pulse_time) * 1e-6 * self.speed_sound / 2.0
                reset = True if int(button_pressed) == 1 else False
                try:
                    secs = int(stamp_ms) // 1000
                    nanosecs = int(stamp_ms) % 1000 * 1000000
                except:
                    continue

                left_range_msg = Range()
                left_range_msg.header.frame_id = 'left'
                left_range_msg.header.stamp.sec = secs
                left_range_msg.header.stamp.nanosec = nanosecs
                left_range_msg.range = left_distance
                self.left_pub.publish(left_range_msg)

                right_range_msg = Range()
                right_range_msg.header.frame_id = 'right'
                right_range_msg.header.stamp.sec = secs
                right_range_msg.header.stamp.nanosec = nanosecs
                right_range_msg.range = right_distance
                self.right_pub.publish(right_range_msg)

                reset_msg = Bool()
                reset_msg.data = reset
                self.reset_pub.publish(reset_msg)
        
def main():
    rclpy.init()
    sensor_reader = SensorReader('/dev/ttyACM0', 9600)
    try:
        sensor_reader.run()
    finally:
        sensor_reader.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()