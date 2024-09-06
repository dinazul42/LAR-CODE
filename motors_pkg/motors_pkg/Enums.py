from enum import Enum

class TorqeedoMotorCmdEnum(Enum):
    SET_SPEED_ID = 0x82
    THRUSTER_ID = 0x00
    THRUSTER_ERROR_ID = 0x01
    THRUSTER_STATUS_ID = 0x03

class TorqeedoMotorSideEnum(Enum):
    LEFT_THR = 0x00
    RIGHT_THR = 0x01

class TorqeedoMotorConstantsEnum(Enum):
    MAX_FORWARD_SPEED = 1000
    MAX_BACKWARD_SPEED = -1000
    SPEED_PACKETS_NUMBER = 4
    STATUS_PACKETS_NUMBER = 2
    FULL_PACKET_LENGTH = 17
    SMALL_PACKET_LENGTH = 5
