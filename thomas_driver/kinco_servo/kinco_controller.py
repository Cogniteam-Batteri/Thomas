from kinco_servo import KincoServo
from kinco_enum import *
import time
class KincoController:
    def __init__(self, port, baudrate=38400, node_id=1):
        self.servo = KincoServo(port,baudrate,node_id)

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
        return self.servo.write_parameter(InvertDir.NAME.value, 0, value)

    def set_target_position(self,value):
        print(f" set_target_position with value {value}")
        return self.servo.write_parameter(TargetPosition.NAME.value, 0, value)

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
        
    def absolute_position(self,target_position,direaction=InvertDir.CW.value,
                            profile_speed=500,profile_acc=610,profile_dec=610):
        
        print(f" absolute_position !!")
        if not self.set_control_word(ControlWord.ERROR_RESET.value):
            print("error in : set_control_word")

        if not self.set_operation_mode(OperationMode.POSITION_CONTROL.value):
            print("error in : set_operation_mode")
        if not self.set_invert_dir(direaction):
            print("error in : set_invert_dir")
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

    def quick_stop(self):
        print(f" quick_stop !!")
        self.set_quick_stop_mode(QuickStopMode.STOP_WITHOUT_CONTROL.value)
        self.set_control_word(ControlWord.ENABLE.value)
        self.set_control_word(ControlWord.QUICK_STOP.value)

    def homing(self):
        print(f" homing !!")
        self.set_operation_mode(OperationMode.HOMING_MODE.value)
        self.set_control_word(ControlWord.ERROR_RESET.value) # clear error
        self.set_control_word(ControlWord.ENABLE.value) #F
        self.set_control_word(ControlWord.HOMEING_SEND_COMMAND.value) #1F

    def deg_to_count(self, deg):

        return deg * ENDCODER_RESOLUTION
        
def main():
    controller = KincoController("/dev/ttyUSB0")  # Adjust port as needed
    
    # controller.homing()
    
    #CW : LEFT
    #CCW RIGHT
    controller.absolute_position(controller.deg_to_count(90),0, 1500, 610, 610)


    # time.sleep(5)
    # controller.quick_stop()
    # time.sleep(5)


    # controller.absolute_position(controller.deg_to_count(10),0, 400)


    #controller.set_profile_speed(200) # ndeeds to be 01 23 81 60 00 55 55 08 00 49


if __name__ == "__main__":
    main()

# cogniteam1!1!
