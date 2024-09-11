

from thomas_driver.kinco_servo.kinco_servo import KincoServo
from thomas_driver.kinco_servo.kinco_enum import *
import math

import time

class KincoController:
    def __init__(self, port, baudrate=38400, node_id=1):
        self.servo = KincoServo(port,baudrate,node_id)
        self.set_control_word(ControlWord.ERROR_RESET.value)


    # def get_status_word(self):
    #     return self.servo.read_parameter(0x4160, 0)  # Note the byte swap here

    def get_control_word(self):
        return self.servo.read_parameter(ControlWord.NAME.value, 0)  # Note the byte swap here

    def get_operation_mode(self):
        return self.servo.read_parameter(OperationMode.NAME.value, 0)  # Note the byte swap here


    def set_operation_mode(self, mode):
        return self.servo.write_parameter(OperationMode.NAME.value, 0, mode)

    def set_control_word(self,value):
        print(f" set_control_word with value {value}")
        return self.servo.write_parameter(ControlWord.NAME.value, 0, value)
    
    def set_quick_stop_mode(self,value):
        print(f" set_quick_stop_mode with value {value}")
        return self.servo.write_parameter(QuickStopMode.NAME.value, 0, value)

    def set_invert_dir(self,value):
        print(f" set_invert_dir with value {value}")
        return self.serdvo.write_parameter(InvertDir.NAME.value, 0, value)

    def set_target_position(self,value):
        print(f" set_target_position with value {value}")
        return self.servo.write_parameter(TargetPosition.NAME.value, 0, value)
    
    def set_target_speed(self,value):
        print(f" set_target_speed with value {value}")
        return self.servo.write_parameter(TargetSpeed.NAME.value, 0, self.calc_rpm_velocity(value))

    def set_profile_speed(self,value):
        print(f" set_profile_speed with value {value}")
        return self.servo.write_parameter(ProfileSpeed.NAME.value, 0, self.calc_rpm_velocity(value))

    def set_profile_acc(self,value):
        print(f" set_profile_acc with value {value}")
        return self.servo.write_parameter(ProfileACC.NAME.value, 0, self.calc_acc_dec(value))

    def set_profile_dec(self,value):
        print(f" set_profile_dec with value {value}")
        return self.servo.write_parameter(ProfileDEC.NAME.value, 0, self.calc_acc_dec(value))

    def uint32(self,value):
        # Mask with 0xFFFFFFFF to keep it within the 32-bit range
        return value & 0xFFFFFFFF
    

    def calc_rpm_velocity(self, rpm_dec):

        try:
            return int((rpm_dec * 512 * ENDCODER_RESOLUTION) / 1875)
        except Exception as e:
            print(f"Unexpected error: {e}")
            return 0



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
        
    def absolute_position(self,target_position,profile_speed=500,profile_acc=610,profile_dec=610):
        

        operation_mode = self.get_operation_mode()
        control_word =   self.get_control_word()


        if operation_mode == OperationMode.POSITION_CONTROL and control_word == ControlWord.ABSOLUTE_ENABLE:
            self.set_target_position(target_position)
            return
        

        print(f" absolute_position !! { target_position}")
        
        if not self.set_operation_mode(OperationMode.POSITION_CONTROL.value):
            print("error in : set_operation_mode")
       
        if not self.set_target_position(target_position):
            print("error in : set_target_position")
        if not self.set_profile_speed(self.uint32(profile_speed)):
            print("error in : set_profile_speed")
        if not self.set_profile_acc(self.uint32(profile_acc)):
            print("error in : set_profile_acc")
        if not self.set_profile_dec(self.uint32(profile_dec)):
            print("error in : set_profile_dec")
        if not self.set_control_word(ControlWord.ABSOLUTE_ENABLE.value):
            print("error in : set_control_word")
        if not self.set_control_word(ControlWord.SEND_COMMAND.value):
            print("error in : set_control_word")

   

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
        print(f" the rpm is {rpm},  the steering_angle_velocity {steering_angle_velocity}")
       
        # Check if the angle is not within the range
        if angle_deg < STEERING_DEG_MAX and angle_deg > STEERING_DEG_MIN:

            self.absolute_position(self.deg_to_count(int(angle_deg)), rpm, 1000, 1000)
            
        else:
            print(f"wanted angle {angle_deg} not in range !!")

    def quick_stop(self):
        print(f" quick_stop !!")
        self.set_quick_stop_mode(QuickStopMode.STOP_BY_USING_RAMP.value)
        self.set_control_word(ControlWord.ENABLE.value)
        self.set_control_word(ControlWord.QUICK_STOP.value)

    def linear_velocity_cmd(self, linear_velocity = 0.5, profile_acc=610,profile_dec=610):

        operation_mode = self.get_operation_mode()
        control_word =   self.get_control_word()

        print(f" the control world is {control_word}, the operation_mode is {operation_mode}")
        if operation_mode == OperationMode.SPEED_CONTROL.value and control_word == ControlWord.ENABLE.value:
            print(f" just update the speed to {linear_velocity}")
            self.set_target_speed(self.wheel_angle_velocity_to_rpm(linear_velocity))
            return   

        print(f" initalize  speed with  {linear_velocity}")
        self.set_control_word(ControlWord.ERROR_RESET.value)

        self.set_operation_mode(OperationMode.SPEED_CONTROL.value)
       
        self.set_target_speed(self.wheel_angle_velocity_to_rpm(linear_velocity))
        self.set_profile_acc(self.uint32(profile_acc))
        self.set_profile_dec(self.uint32(profile_dec))

        self.set_control_word(ControlWord.ENABLE.value)

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
        print(f" the rpm isssssssssssssssssssss {rpm}")
        return rpm

# cogniteam1!1!
