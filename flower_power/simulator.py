import rclpy
from rclpy.node import Node
from message_filters import TimeSynchronizer, Subscriber
from sensor_msgs.msg import Range
from rclpy.qos import qos_profile_sensor_data

from .vehicle import OnlineVehicle, PrecomputedVehicle
from geometry_msgs.msg import Pose

class Simulator(Node):
    def __init__(self, dt=0.25):
        super().__init__('simulator')
        # self.server = Server('localhost', 12345)
        self.vehicles = {'target': PrecomputedVehicle(dt)}
                        #  'ego': OnlineVehicle(self.dt)}
        self.timer = self.create_timer(dt, self.tick_callback)
        tss = TimeSynchronizer([Subscriber(self, Range, 'range_left', qos_profile=qos_profile_sensor_data),
                                Subscriber(self, Range, 'range_right', qos_profile=qos_profile_sensor_data)],
                                10)
        # tss.registerCallback(self.sensor_callback)
        self.pubs = {'target': self.create_publisher(Pose, 'target/pose', 10)}

    def tick_callback(self):
        self.vehicles['target'].step()
        for name, vehicle in self.vehicles.items():
            pose_msg = vehicle.get_state()
            # self.get_logger().info(str(pose_msg))
            self.pubs[name].publish(pose_msg)
    
    def sensor_callback(self, left_msg, right_msg):
        self.vehicles['ego'].update(left_msg.range, right_msg.range)
        self.vehicles['ego'].step()

def main():
    rclpy.init()
    sim = Simulator()
    rclpy.spin(sim)
    

if __name__ == '__main__':
    main()
