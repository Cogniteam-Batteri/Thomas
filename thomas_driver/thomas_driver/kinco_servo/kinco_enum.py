from enum import Enum

ENDCODER_RESOLUTION = 10000

# Define the range
STEERING_DEG_MIN = -119
STEERING_DEG_MAX = 119

# Define an Enum class
class OperationMode(Enum):
    NAME = 0x6060
    POSITION_CONTROL = 1
    HOMING_MODE = 6
 
# Define an Enum class
class ControlWord(Enum):
    NAME = 0x6040
    ERROR_RESET = 0x86
    ENABLE = 0X0F
    HOMEING_SEND_COMMAND = 0x1F
    ABSOLUTE_ENABLE = 0X2F
    QUICK_STOP = 0X0b
    SEND_COMMAND = 0x3F

class QuickStopMode(Enum):
    NAME = 0x605A
    STOP_WITHOUT_CONTROL = 0
    STOP_BY_USING_RAMP = 1
    STOP_BY_USING_QUICK_STOP_DECELERATION = 2
    STOP_WITH_PROFILE_DECELERATION = 5
    STOP_BY_USING_QUICK_STOP_DECELERATION_QUICK_STOP_ACTIVE = 6 
    MONITOR_WINDING = 18


class InvertDir(Enum):
    NAME = 0x607E
    CCW = 0x01
    CW = 0x00

class TargetPosition(Enum):
    NAME = 0x607A

class ProfileSpeed(Enum):
    NAME = 0x6081

class ProfileACC(Enum):
    NAME = 0x6083

class ProfileDEC(Enum):
    NAME = 0x6084