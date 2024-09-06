"""This file contains all public Enums, constants, and definitions needed for the project."""
from enum import IntEnum, auto

class ParametersID(IntEnum):
    """Parameters enum."""
    CRUISE_SPEED_ID = auto()
    CRUISE_MIN_SPEED_ID = auto()
    MAXBAT_VOLTAGE_ID = auto()
    MINBAT_VOLTAGE_ID = auto()
    LIMIT_DEPTH_ID = auto()
    LIMIT_TIME_ID = auto()
    PID_V_PITCH_KP_ID = auto()
    PID_V_PITCH_KD_ID = auto()
    PID_V_PITCH_KI_ID = auto()
    PID_V_PITCH_KI_MAX_ID = auto()
    PID_V_PITCH_MIN_ID = auto()
    PITCH_V_MAX_ID = auto()
    PID_V_DEPTH_KP_ID = auto()
    PID_V_DEPTH_KD_ID = auto()
    PID_V_DEPTH_KI_ID = auto()
    PID_V_DEPTH_KI_MAX_ID = auto()
    PID_V_ROLL_KP_ID = auto()
    PID_V_ROLL_KD_ID = auto()
    PID_V_ROLL_KI_ID = auto()
    PID_V_ROLL_KI_MAX_ID = auto()
    PID_V_HDG_KP_ID = auto()
    PID_V_HDG_KD_ID = auto()
    PID_V_HDG_KI_ID = auto()
    PID_V_HDG_KI_MAX_ID = auto()
    PID_V2_HDG_KP_ID = auto()
    PID_V2_HDG_KD_ID = auto()
    PID_V2_HDG_KI_ID = auto()
    PID_V2_HDG_KI_MAX_ID = auto()
    PID_V3_HDG_KP_ID = auto()
    PID_V3_HDG_KD_ID = auto()
    PID_V3_HDG_KI_ID = auto()
    PID_V3_HDG_KI_MAX_ID = auto()
    PID_V2_PITCH_KP_ID = auto()
    PID_V2_PITCH_KD_ID = auto()
    PID_V2_PITCH_KI_ID = auto()
    PID_V2_PITCH_KI_MAX_ID = auto()
    PID_V2_PITCH_MIN_ID = auto()
    PITCH_V2_MAX_ID = auto()
    PID_V2_DEPTH_KP_ID = auto()
    PID_V2_DEPTH_KD_ID = auto()
    PID_V2_DEPTH_KI_ID = auto()
    PID_V2_DEPTH_KI_MAX_ID = auto()
    PID_V2_ROLL_KP_ID = auto()
    PID_V2_ROLL_KD_ID = auto()
    PID_V2_ROLL_KI_ID = auto()
    PID_V2_ROLL_KI_MAX_ID = auto()
    PID_V3_PITCH_KP_ID = auto()
    PID_V3_PITCH_KD_ID = auto()
    PID_V3_PITCH_KI_ID = auto()
    PID_V3_PITCH_KI_MAX_ID = auto()
    PID_V3_PITCH_MIN_ID = auto()
    PITCH_V3_MAX_ID = auto()
    PID_V3_DEPTH_KP_ID = auto()
    PID_V3_DEPTH_KD_ID = auto()
    PID_V3_DEPTH_KI_ID = auto()
    PID_V3_DEPTH_KI_MAX_ID = auto()
    PID_V3_ROLL_KP_ID = auto()
    PID_V3_ROLL_KD_ID = auto()
    PID_V3_ROLL_KI_ID = auto()
    PID_V3_ROLL_KI_MAX_ID = auto()
    WINDUP_HDG_ID = auto()
    WINDUP_HDG_GAIN_ID = auto()
    PARAM_INS_EULER_RST_ID = auto()
    PARAM_INS_POS_RST_ID = auto()
    WING_LEFT_ZERO_ID = auto()
    WING_RIGHT_ZERO_ID = auto()
    WING_UPPER_ZERO_ID = auto()
    WING_SERVO_OFFSET_ID = auto()
    TARGET_Z_ID = auto()
    SET_SPEED_ID = auto()
    STEP_MAIN_MOTOR_ID = auto()
    SD_NUM_ID = auto()
    LIMIT_MAX_PRSR_ID = auto()
    CRUISE_SPEED_FACT_ID = auto()
    WING_UPPER_OFF_ID = auto()
    WING_DIVE_POS_ID = auto()
    DIVE_CNST_DTH_ID = auto()
    DIVE_CNST_WP_ID = auto()
    SET_THR_DELAY_ID = auto()
    SET_START_SPD_ID = auto()
    SHUTDOWN_ID = auto()
    ONBOARD_PARAM_COUNT = auto()  # do not touch. Do not put values below this. You may add on top of it.

class KayakMode(IntEnum):
    """Kayak mode enum."""
    Kayak_IDLE = 1  # idle (pause button)
    Kayak_JOYSTICK = 2  # manual control by joystick (pause)
    Kayak_FIXED_MANEUVER = 4  # doing one fixed maneuver based on param (pause)
    Kayak_HOLD_POSITION = 8  # stops go to home (stop button)
    Kayak_ABORTED_ONLY_MANUAL = 16  # stop and don't use the hold position (stop)
    Kayak_ABORTED_LPM = 32  # stopped (no battery) only enable comm, NO MOTORS (stop)
    Kayak_CALIBRATION = 64  # in magnetometer calibration mode # not used!
    Kayak_GUIDED_MISSION = 128  # in wp mission mode mode (play)

class KayakStatus(IntEnum):
    """Kayak status enum."""
    Kayak_MISSION_IDLE = 1  # on surface on manual mode
    Kayak_DIVE = 2  # start diving with depth error higher than 2 and wait_time > 0
    Kayak_MISSION = 4  # on way to next waypoint
    Kayak_RISING_GPS = 8  # on its way up for gps measurement
    Kayak_RISING_FINISH = 16  # got to surface and awaiting to gps_fix > 1
    Kayak_WAIT_GPS = 32  # got gps_fix wait for gps_timer to finish
    Kayak_ABORT = 64  # ABORT
    Kayak_PREMISSION = 128
    Kayak_FINISH = 256

class QGCMode(IntEnum):
    """QGC set_mode enum."""
    PREFLIGHT = 0
    PLAY = 28
    STOP = 32
    PAUSE = 64

class HardLimits(IntEnum):
    """Hard coded parameters."""
    Max_BAT_Voltage = 40  # [V]
    Min_BAT_Voltage = 25  # [V]
    Max_Humidity = 70  # Percent
    Max_Temperature = 55  # Celsius
    Max_Pressure = 1.2  # bar

class ErrorCode(IntEnum):
    """
    Error codes for sending to QGC.

    A full match is required between the values in the code
    and the error_codes_HC200.ini file located in the QGC folder
    """
    No_Error = 0
    Bat_Voltage_Error = 1
    BME_Error = 2
    Mission_Done = 4
    Distance_less_than_30m = 8
    Mission_Timeout_Error = 16
    GPS_Connection_Error = 32
