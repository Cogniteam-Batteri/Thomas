import pygame
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String

from enum import Enum
import yaml
import math

class XBOX_1_BUTTON(Enum):
    Y_button = 4
    B_button = 1
    A_button = 0
    X_button = 3

# 0: left stick : x axis (left side -1 right side 1)
# 1: left stick : y axis (up side -1 down side 1)
# 2: right stick : x axis (left side -1 right side 1)
# 3  right stick : y axis (up side -1 down side 1)
class XBOX_1_STICK(Enum):
    LEFT_X_AXIS =  0
    LEFT_Y_AXIS =  1
    RIGHT_X_AXIS = 2
    RIGHT_Y_AXIS = 3


THRESHOLD_LINEAR_VEL = 0.1
   
class ThomasJoystickController(Node):
    def __init__(self):
        super().__init__('thomas_joystick_controller_node')
        
        self.cmd_vel_publisher = self.create_publisher(AckermannDriveStamped, '/thomas/cmd_vel', 10)
        self.driver_state_publisher = self.create_publisher(String, '/thomas/mode_switch', 10)        

        self.load_params('/home/simulation/battery_ws/src/Thomas/thomas_joystick_controller/config/config.yaml')

        self.load_joystick()
        
        # Set up timer based on frequency (hz)
        self.timer = self.create_timer(1.0 / self.hz, self.timer_callback)


    def load_joystick(self):

        self.is_joystick_init = False

        try:
            # Initialize pygame and the joystick
            pygame.init()
            pygame.joystick.init()

            # Ensure at least one joystick is connected
            if pygame.joystick.get_count() == 0:
                self.get_logger().error('No joystick connected')
                exit()

            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.is_joystick_init = True
            self.get_logger().info('The joystick has been successfully initialized!')
        
        except Exception as e:
            self.get_logger().error(f'Failed to connect to joystick: {str(e)}')
            exit(-1)

    def load_params(self, yaml_path ):

        # Assuming the YAML content is in a file called 'config.yaml'
        with open(yaml_path, 'r') as file:
            config_dict = yaml.safe_load(file)
        
        self.hz = float(config_dict['hz'])
        self.min_max_linear_velocity = float(config_dict['min_max_linear_velocity'])
        self.max_steering_angle_deg = float(config_dict["max_steering_angle_deg"])
        self.steering_angle_velocity = float(config_dict["steering_angle_velocity"])

         # Create a dictionary that maps names to the corresponding class methods
        self.actions_map = {
            'HOMING': self.homing_action,
            'RESET_ERRORS': self.reset_error_action,
            'JOYSTICK': self.joystick_action,
            'IDLE': self.idle_action
        }

        self.buttons_actions = {}
        try:
            self.buttons_actions[str(XBOX_1_BUTTON.Y_button.value)] = self.actions_map[config_dict[XBOX_1_BUTTON.Y_button.name]]
            self.buttons_actions[str(XBOX_1_BUTTON.B_button.value)] = self.actions_map[config_dict[XBOX_1_BUTTON.B_button.name]]     
            self.buttons_actions[str(XBOX_1_BUTTON.A_button.value)] = self.actions_map[config_dict[XBOX_1_BUTTON.A_button.name]]  
            self.buttons_actions[str(XBOX_1_BUTTON.X_button.value)] = self.actions_map[config_dict[XBOX_1_BUTTON.X_button.name]] 


            print(f" self.buttons_actions { self.buttons_actions}")   
        except Exception as e:
            self.get_logger().error(f"{e}")
 

       
    def reset_error_action(self):
        print("reset_errors action triggered!!!!!!!!!!!!!!!!!!!!")
        msg = String()
        msg.data = 'reset_errors'
        self.driver_state_publisher.publish(msg)


     
    def homing_action(self):
        print("Homing action triggered!!!!!!!!!!!!!!!!!!!!")
        msg = String()
        msg.data = 'homing'
        self.driver_state_publisher.publish(msg)

    def joystick_action(self):
        print("Joystick action triggered!")
        msg = String()
        msg.data = 'joystick'
        self.driver_state_publisher.publish(msg)

    def discrete_action(self):
        print("Discrete action triggered!")
        msg = String()
        msg.data = 'discrete'
        self.driver_state_publisher.publish(msg)


    def idle_action(self):
        print("Idle action triggered!")
        msg = String()
        msg.data = 'idle'
        self.driver_state_publisher.publish(msg)


    def create_action_cmd(self):

        # Get button presses
        for button_value in range(self.joystick.get_numbuttons()):
            button_state = self.joystick.get_button(button_value)
            if button_state:
                self.get_logger().info(f'Button {button_value} pressed')
                try:                    
                    self.buttons_actions[str(button_value)]()
                    return True
                except Exception as e:    
                    self.get_logger().error(f"{e}")
                    return False
        return False
    

    def create_linear_velocity(self):

        linear_velocity = 0.0
        # check that the right X axis not pressed 
        if math.fabs(self.joystick.get_axis(XBOX_1_STICK.RIGHT_X_AXIS.value)) < 0.9:
            direction = 1.0
            # reverse
            if self.joystick.get_axis(XBOX_1_STICK.RIGHT_Y_AXIS.value) > 0:
                direction = -1.0
            
            linear_velocity = direction * self.normalize_linear_velocity(math.fabs(self.joystick.get_axis(XBOX_1_STICK.RIGHT_Y_AXIS.value)), 
                                                                               0, self.min_max_linear_velocity)        
            # threshold stick velocity
            if math.fabs(linear_velocity) < THRESHOLD_LINEAR_VEL:
                linear_velocity = 0.0

        return  linear_velocity     


    def create_steering_angle(self):
        
        #radians
        steering_angle = 0.0
        #radians/s
        steering_angle_velocity = math.radians(self.steering_angle_velocity)

        wanted_deg  = -1 * self.max_steering_angle_deg * self.joystick.get_axis(XBOX_1_STICK.LEFT_X_AXIS.value)

        # if math.fabs(wanted_deg) > 30:
        #     steering_angle_velocity = math.radians(20)

        steering_angle = math.radians(wanted_deg)

        

        return steering_angle, steering_angle_velocity
      

    def create_ackermann_cmd(self):       
            
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'  
        msg.drive.speed = self.create_linear_velocity()  # Speed in m/s
        msg.drive.steering_angle, msg.drive.steering_angle_velocity = self.create_steering_angle()

        self.cmd_vel_publisher.publish(msg)
        
        
        
        # # -1 FORAWRD
        # #  1 BACKWARD

    def normalize_linear_velocity(self, value, min_linear_vel, max_linear_vel):
        
        if not (0 <= value <= 1):
            raise ValueError("Input value must be between 0 and 1.")

        normalized_value = min_linear_vel + (max_linear_vel - min_linear_vel) * value
        return normalized_value


    def timer_callback(self):
        if not self.is_joystick_init:
            self.get_logger().warn('Joystick is not initialized.')
            return

        # Handle event processing (necessary for pygame to get joystick updates)
        pygame.event.pump()

        if self.create_action_cmd() == False:
                
            self.create_ackermann_cmd()
        


def main(args=None):
    rclpy.init(args=args)
    node = ThomasJoystickController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
