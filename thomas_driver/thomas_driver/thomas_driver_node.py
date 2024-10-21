import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from ackermann_msgs.msg import AckermannDriveStamped
from enum import Enum
from diagnostic_updater import Updater, DiagnosticTask
from diagnostic_msgs.msg import DiagnosticStatus
from thomas_driver.kinco_servo.kinco_controller import KincoController
from thomas_driver.kinco_servo.kinco_enum import *

import math

class DrivingMode(Enum):
    IDLE =      'idle'
    HOMING =    'homing'
    DISCRETE =  'discrete'
    JOYSTICK =  'joystick'

class ThomasDriver(Node):
    def __init__(self):
        super().__init__('thomas_driver_node')

        # Initialize the steering_controller
        self.steering_controller = KincoController(MotorType.STEERING,"/dev/steering_motor")  # TOP

        self.wheel_controller = KincoController(MotorType.WHEEL,"/dev/wheel_motor") # DOWN



        # Initialize mode state
        self.mode = DrivingMode.IDLE
        self.angle = 0
        self.prev_steering_angle = 0.0
        self.prev_linear_vel = 0.0

        # Create subscribers
        self.create_subscription(String, '/thomas/mode_switch', self.mode_switch_callback, 10)
        self.create_subscription(String, '/thomas/angle_cmd', self.angle_callback, 10)
        self.create_subscription(String, '/thomas/velocity_cmd', self.velocity_callback, 10)

        self.create_subscription(AckermannDriveStamped, '/thomas/cmd_vel', self.joystick_callback, 10)

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


    def stop_wheel(self):

        self.wheel_controller.linear_velocity_cmd(0)       

    def joystick_callback(self, msg):      

        # diff_deg = self.absolute_diff_in_degrees(msg.drive.steering_angle, self.prev_steering_angle)
        # self.prev_steering_angle = msg.drive.steering_angle       

        if self.mode == DrivingMode.JOYSTICK:
            
            # STOP
            if msg.drive.speed == 0.0:
                self.wheel_controller.linear_velocity_cmd(msg.drive.speed)  
            else:
                # DRIVING
                self.wheel_controller.linear_velocity_cmd(msg.drive.speed)    

            if msg.drive.steering_angle == 0.0:
                # Re-align the vehicle's steering motor
                
                # self.steering_controller.quick_stop()

                self.steering_controller.angle_position(0, msg.drive.steering_angle_velocity)     
            
            else:
                self.steering_controller.angle_position(math.degrees(msg.drive.steering_angle), msg.drive.steering_angle_velocity)

    
    def mode_switch_callback(self, msg: String):
        try:
            if msg.data == 'homing' and self.mode != DrivingMode.HOMING:
                self.mode = DrivingMode.HOMING
                self.stop_wheel()
                self.steering_controller.homing()
            elif msg.data == 'reset_errors':
                self.steering_controller.reset_errors()
                self.wheel_controller.reset_errors()
                self.mode = DrivingMode.IDLE
            elif msg.data == 'discrete' and self.mode != DrivingMode.DISCRETE:
                self.mode = DrivingMode.DISCRETE
            elif msg.data == 'joystick' and self.mode != DrivingMode.JOYSTICK:
                self.mode = DrivingMode.JOYSTICK    
            elif msg.data == 'idle' and self.mode != DrivingMode.IDLE:
                self.mode = DrivingMode.IDLE    
            
            self.get_logger().info(f"the mode is: {msg.data}")
               
        except KeyError:
            self.get_logger().warn(f'Invalid mode: {msg.data}')


    def velocity_callback(self, msg: String):

        self.get_logger().info(f'Received vel: {float(msg.data)}')
        self.wheel_controller.linear_velocity_cmd(float(msg.data))


    def angle_callback(self, msg: String):
        steering_angle_velocity = 10 * (math.pi / 180)

        angle = int(msg.data)
        self.get_logger().info(f'Received angle: {angle}')
        self.steering_controller.angle_position(angle, steering_angle_velocity)

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
