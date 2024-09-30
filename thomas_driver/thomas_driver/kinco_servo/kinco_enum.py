from enum import Enum

# Define a new Enum class for data types
class DataType(Enum):
    INT8 = 'int8'  # 1 byte
    INT16 = 'int16' # 2 bytes
    INT32 = 'int32' # 4 bytes
    UNSIGNED_INT16 = 'unsignedint16'  # 2 bytes
    UNSIGNED_INT32 = 'unsignedint32'  # 4 bytes


class MotorType(Enum):
    STEERING = 1
    WHEEL =    2
   

# Encoder and motor settings
ENDCODER_RESOLUTION = 10000
WHEEL_DIAMETER =  0.32
GEAR_RATIO = 30
DIFF_LINEAR_VEL_THRESHOLD = 0.01
DIFF_STEERING_ANGLE_DEGREE_THRESHOLD = 1


DEFUALT_PROFILE_SPEED = 1000
DEFUALT_PROFILE_ACC =   610
DEFUALT_PROFILE_DEC =   610

# Define the range
STEERING_DEG_MIN = -119
STEERING_DEG_MAX = 119

# Define an Enum class for operation modes
class OperationMode(Enum):
    NAME = (DataType.INT8, 0x6060)
    POSITION_CONTROL = 1
    SPEED_CONTROL = 3
    HOMING_MODE = 6 # 01:2f:60:60:00:06:0a not working
    
 
# Define an Enum class for control words
class ControlWord(Enum):
    NAME = (DataType.UNSIGNED_INT16, 0x6040) 
    ERROR_RESET = 0x86
    ENABLE = 0x0F  # give power to the motor
    DISABLE = 0x06  # disable power to the motor
    HOMING_SEND_COMMAND = 0x1F
    ABSOLUTE_ENABLE = 0x2F
    QUICK_STOP = 0x0B
    SEND_COMMAND = 0x3F
    SEND_RUNNING_COMMAND = 0x103F

# Define an Enum class for quick stop modes
class QuickStopMode(Enum):
    NAME = (DataType.UNSIGNED_INT16, 0x605A)
    STOP_WITHOUT_CONTROL = 0
    STOP_BY_USING_RAMP = 1
    STOP_BY_USING_QUICK_STOP_DECELERATION = 2
    STOP_WITH_PROFILE_DECELERATION = 5
    STOP_BY_USING_QUICK_STOP_DECELERATION_QUICK_STOP_ACTIVE = 6 
    MONITOR_WINDING = 18

# Define an Enum class for target position
class TargetPosition(Enum):
    NAME = (DataType.INT32, 0x607A)

# Define an Enum class for target speed
class TargetSpeed(Enum):
    NAME = (DataType.INT32, 0x60FF)

# Define an Enum class for real speed
class RealSpeed(Enum):
    NAME = (DataType.INT32, 0x606C)

# Define an Enum class for actual pose
class ActualPose(Enum):
    NAME = (DataType.INT32, 0x6063)

# Define an Enum class for profile speed
class ProfileSpeed(Enum):
    NAME = (DataType.UNSIGNED_INT32, 0x6081)

# Define an Enum class for profile acceleration
class ProfileACC(Enum):
    NAME = (DataType.UNSIGNED_INT32, 0x6083)

# Define an Enum class for profile deceleration
class ProfileDEC(Enum):
    NAME = (DataType.UNSIGNED_INT32, 0x6084)
