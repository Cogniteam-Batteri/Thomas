import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from ackermann_msgs.msg import AckermannDriveStamped
from enum import Enum
from diagnostic_updater import Updater, DiagnosticTask
from diagnostic_msgs.msg import DiagnosticStatus
from thomas_driver.kinco_servo.kinco_controller import KincoController
import math

class DrivingMode(Enum):
    HOMING = 'homing'
    DISCRETE = 'discrete'
    JOYSTICK = 'joystick'

class ThomasDriver(Node):
    def __init__(self):
        super().__init__('thomas_driver_node')

        # Initialize the steering_controller
        self.steering_controller = KincoController("/dev/ttyUSB0")  
        self.steering_angle_velocity = 25 * (math.pi / 180)


        # Initialize mode state
        self.mode = DrivingMode.DISCRETE
        self.angle = 0
        self.prev_steering_angle = 0.0

        # Create subscribers
        self.create_subscription(String, 'mode_switch', self.mode_switch_callback, 10)
        self.create_subscription(String, '/thomas/angle_cmd', self.angle_callback, 10)
        self.create_subscription(Bool, '/thomas/stop', self.stop_callback, 10)
        self.create_subscription(AckermannDriveStamped, '/thomas/ackermann_cmd', self.joystick_callback, 10)

        # Initialize diagnostic updater
        self.updater = Updater(self)
        self.updater.setHardwareID("ThomasDriver")
        self.updater.add("Driving Mode", self.diagnostic_callback)

        # Set timer to update diagnostics at a regular interval
        self.create_timer(1.0, self.updater.force_update)  # 1 Hz update rate

    def diagnostic_callback(self, stat: DiagnosticTask):
        # Update diagnostic with the current driving mode
        mode_str = f"{self.mode.value}"
        stat.summary(DiagnosticStatus.OK, mode_str)
        return stat

    def joystick_callback(self, msg):
        #  # Extract the header information
        # self.get_logger().info(f'Received AckermannDriveStamped command:')
        # self.get_logger().info(f'  Header: {msg.header}')
        # self.get_logger().info(f'  Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')
        # self.get_logger().info(f'  Frame ID: {msg.header.frame_id}')
        # # Extract the drive command data
        # self.get_logger().info(f'  Steering Angle: {msg.drive.steering_angle}')
        # self.get_logger().info(f'  Steering Angle Velocity: {msg.drive.steering_angle_velocity}')
        # self.get_logger().info(f'  Speed: {msg.drive.speed}')
        # self.get_logger().info(f'  Acceleration: {msg.drive.acceleration}')
        # self.get_logger().info(f'  Jerk: {msg.drive.jerk}')

        diff_deg = self.absolute_diff_in_degrees(msg.drive.steering_angle, self.prev_steering_angle)
        self.prev_steering_angle = msg.drive.steering_angle

        if self.mode == DrivingMode.JOYSTICK:
            if msg.drive.speed == 0.0:
                print('SSSSSSSSSSSSSSSSSSSSSSSSSSSSSS')
                self.steering_controller.quick_stop()
            elif diff_deg > 1:
                self.steering_controller.angle_position(math.degrees(msg.drive.steering_angle), self.steering_angle_velocity)

    def stop_callback(self, msg: Bool):
        self.get_logger().info(f'Received stop signal: {msg.data}')
        if msg.data and self.mode == DrivingMode.DISCRETE:
            self.steering_controller.quick_stop()
            self.get_logger().info('Quick stop executed in DISCRETE mode.')

    def mode_switch_callback(self, msg: String):
        try:
            print(f" the msg.data is {msg.data}")
            if msg.data == 'homing':
                self.mode = DrivingMode.HOMING
            elif msg.data == 'discrete':
                self.mode = DrivingMode.DISCRETE
            elif msg.data == 'joystick':
                self.mode = DrivingMode.JOYSTICK    
            else:
                self.get_logger().info('Invalid mode!.')

            if self.mode == DrivingMode.HOMING:
                self.steering_controller.homing()
                self.get_logger().info('Executing homing procedure.')

        except KeyError:
            self.get_logger().warn(f'Invalid mode: {msg.data}')

    def angle_callback(self, msg: String):
        self.angle = int(msg.data)
        self.get_logger().info(f'Received angle: {self.angle}')
        if self.mode == DrivingMode.DISCRETE:
            self.steering_controller.angle_position(self.angle, self.steering_angle_velocity)
            self.get_logger().info(f'Set angle to: {self.angle} in DISCRETE mode.')

    def absolute_diff_in_degrees(self, rad1, rad2):
        diff_radians = abs(rad1 - rad2)
        diff_degrees = abs(math.degrees(diff_radians))
        return diff_degrees

def main(args=None):
    rclpy.init(args=args)
    node = ThomasDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
