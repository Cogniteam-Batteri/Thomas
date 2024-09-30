

from thomas_driver.kinco_servo.kinco_servo import KincoServo
from thomas_driver.kinco_servo.kinco_enum import *
import math
import time

class KincoController:

    def __init__(self, motor_type, port, baudrate=38400, node_id=1):
        self.servo = KincoServo(port,baudrate,node_id)        
        
        self.set_motor_type(motor_type)

        self.init_motor()

    def set_motor_type(self, motor_type):

        self.motor_type = motor_type

        if self.motor_type == MotorType.STEERING:
            self.set_target_position(0)
        elif self.motor_type == MotorType.WHEEL:
            self.set_profile_speed(0)
    
    def init_motor(self):

     
        self.set_quick_stop_mode(QuickStopMode.STOP_BY_USING_RAMP.value) # ONLY ONE --NED TO CHANGE
        self.set_control_word(ControlWord.DISABLE.value)
        self.set_control_word(ControlWord.ENABLE.value)

        if self.motor_type == MotorType.STEERING:
            self.set_operation_mode(OperationMode.POSITION_CONTROL.value)      
            self.set_profile_speed(self.uint32(DEFUALT_PROFILE_SPEED))
            self.set_profile_acc(self.uint32(DEFUALT_PROFILE_ACC))
            self.set_profile_dec(self.uint32(DEFUALT_PROFILE_DEC))    
            self.set_control_word(ControlWord.SEND_RUNNING_COMMAND.value)

        elif self.motor_type == MotorType.WHEEL:

            # print(f" initalize  speed with  {linear_velocity}")
            self.set_control_word(ControlWord.ERROR_RESET.value)
            self.set_operation_mode(OperationMode.SPEED_CONTROL.value)            
            self.set_target_speed(self.wheel_angle_velocity_to_rpm(0))
            self.set_profile_acc(self.uint32(DEFUALT_PROFILE_ACC))
            self.set_profile_dec(self.uint32(DEFUALT_PROFILE_DEC))
            self.set_control_word(ControlWord.ENABLE.value)


    def get_control_word(self):
        return self.servo.read_parameter(ControlWord.NAME.value[0], ControlWord.NAME.value[1], 0)  # Note the byte swap here

    def get_operation_mode(self):
        return self.servo.read_parameter(OperationMode.NAME.value[0],OperationMode.NAME.value[1] , 0)  # Note the byte swap here

    def get_real_speed(self):
        return self.servo.read_parameter(RealSpeed.NAME.value[0], RealSpeed.NAME.value[1], 0)  # Note the byte swap here

    def get_pose_actual(self):
        return self.servo.read_parameter(ActualPose.NAME.value[0],ActualPose.NAME.value[1],0)
    

    #################################### SETTERS ################################################
    def set_operation_mode(self, mode):
        return self.servo.write_parameter(OperationMode.NAME.value[0], OperationMode.NAME.value[1], 0, mode)

    def set_control_word(self,value):
        return self.servo.write_parameter(ControlWord.NAME.value[0],ControlWord.NAME.value[1], 0, value)
    
    def set_quick_stop_mode(self,value):
        return self.servo.write_parameter(QuickStopMode.NAME.value[0],QuickStopMode.NAME.value[1], 0, value)    

    def set_target_position(self,value):
        return self.servo.write_parameter(TargetPosition.NAME.value[0],TargetPosition.NAME.value[1], 0, value)
    
    def set_target_speed(self,value):
        # print(f" set_target_speed with value {value}")
        return self.servo.write_parameter(TargetSpeed.NAME.value[0],TargetSpeed.NAME.value[1], 0, self.calc_rpm_velocity(value))

    def set_profile_speed(self,value):
        # print(f" set_profile_speed with value {value}")
        return self.servo.write_parameter(ProfileSpeed.NAME.value[0],ProfileSpeed.NAME.value[1], 0, self.calc_rpm_velocity(value))

    def set_profile_acc(self,value):
        # print(f" set_profile_acc with value {value}")
        return self.servo.write_parameter(ProfileACC.NAME.value[0],ProfileACC.NAME.value[1], 0, self.calc_acc_dec(value))

    def set_profile_dec(self,value):
        # print(f" set_profile_dec with value {value}")
        return self.servo.write_parameter(ProfileDEC.NAME.value[0],ProfileDEC.NAME.value[1], 0, self.calc_acc_dec(value))

    def uint32(self,value):
        # Mask with 0xFFFFFFFF to keep it within the 32-bit range
        return value & 0xFFFFFFFF
    
    #################################################################################################################
    def calc_rpm_velocity(self, rpm_dec):

        try:
            return int((rpm_dec * 512 * ENDCODER_RESOLUTION) / 1875)
        except Exception as e:
            print(f"Unexpected error: {e}")
            return 0

    def reset_errors(self):

        self.set_control_word(ControlWord.ERROR_RESET.value)

        self.init_motor()

    def calc_acc_dec(self, rps_s):

        try:            
            return int((rps_s*65536*ENDCODER_RESOLUTION)/1000/4000)
        except Exception as e:
            print(f"Unexpected error: {e}")
            return 0

    def calc_dec(self, rps_s):

        try:
            return int((rps_s * 65536 * ENDCODER_RESOLUTION)) 
        except Exception as e:
            print(f"Unexpected error: {e}")
            return 0
        
    def absolute_position(self,target_position):        

        operation_mode = self.get_operation_mode()
        control_word =   self.get_control_word()


        #if operation_mode == OperationMode.POSITION_CONTROL.value and control_word == ControlWord.SEND_RUNNING_COMMAND.value: 
        self.set_target_position(target_position)
               
       

   

    def homing(self):
        print(f" homing !!")
        self.set_operation_mode(OperationMode.HOMING_MODE.value)
        self.set_control_word(ControlWord.ENABLE.value) #F
        self.set_control_word(ControlWord.HOMING_SEND_COMMAND.value) #1F

    def deg_to_count(self, deg):
        # יחס 1:362 בסיבוב.
        # return deg * ENDCODER_RESOLUTION
        return int((362 / 360) * deg * ENDCODER_RESOLUTION)


    def get_absolute_position(self):
        res = self.servo.read_parameter(0x63,0x0020)                               
        print(f" the res is  {res}")

    def angle_position(self, angle_deg, steering_angle_velocity = 0.78):
        
        rpm = self.steering_angle_velocity_to_rpm(steering_angle_velocity)
       
        # Check if the angle is not within the range
        if angle_deg < STEERING_DEG_MAX and angle_deg > STEERING_DEG_MIN:

            self.absolute_position(self.deg_to_count(int(angle_deg)))
            
        else:
            print(f"wanted angle {angle_deg} not in range !!")

    def quick_stop(self):
        
        # if no need  quick_stop
        current_speed = self.get_real_speed()
        if current_speed == 0:
            return
        
        self.set_control_word(ControlWord.QUICK_STOP.value)
        self.set_control_word(ControlWord.ENABLE.value) 

        if self.motor_type == MotorType.STEERING:
            while (True):
                current_speed = self.get_real_speed()
                if current_speed == 0:
                    pose_actual = self.get_pose_actual()
                    print(f" the pose_actual is ..................{pose_actual}")
                    self.set_target_position(pose_actual)
                    print('afterrrrrrrrrrrrrrrr ')
                    self.init_motor()
                    return
                time.sleep(0.1)
    
    def linear_velocity_cmd(self, linear_velocity = 0.5):    
        # if operation_mode == OperationMode.SPEED_CONTROL.value and control_word == ControlWord.ENABLE.value:
            # print(f" just update the speed to {linear_velocity}")
        self.set_target_speed(self.wheel_angle_velocity_to_rpm(linear_velocity))
               

        

    def steering_angle_velocity_to_rpm(self, steering_angle_velocity):
         # Encapsulated constants
        rpm_initial = 1500
        angle_change = math.pi / 2  # 90 degrees in radians
        time_seconds = 5
        
        # Compute the initial rate of angle change
        initial_rate = angle_change / time_seconds
        
        # Calculate and return the new R based on rad_per_second
        return int(rpm_initial * (steering_angle_velocity / initial_rate))

    def wheel_angle_velocity_to_rpm(self, linear_velocity):       
        
        rpm =  (linear_velocity * 60 * GEAR_RATIO) / (WHEEL_DIAMETER * math.pi)
        return rpm

