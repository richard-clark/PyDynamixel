"""
Registers
=========

Contains constants which correspond to registers, as well as constants
for certain register values, and default register values.

"""

# The model number of the servo.
MODEL_NUMBER = 0x00

# The firmware version of the servo.
VERSION = 0x02

# The identifier of the servo.
ID = 0x03,

# The baud rate at which the servo communicates.
BAUD_RATE = 0x04,

# The delay between when a servo receives a packet and when it sends a response.
RETURN_DELAY = 0x05,

CW_ANGLE_LIMIT = 0x06
CCW_ANGLE_LIMIT = 0x08

# The maximum temperature (units are degrees Celsius).
TEMP_LIMIT = 0x0B

# Upper and lower voltage limits. Units are 100mV (for example, 80 = 8V).
MIN_VOLTAGE_LIMIT = 0x0C
MAX_VOLTAGE_LIMIT = 0x0D

# Torque is relative to the maximum possible torque (so 1023 corresponds to 100%
# of maximum possible torque; 512 corresponds to 50% of maximum torque).
MAX_TORQUE = 0x0E

# Sets status packet behavior.
STATUS_RETURN_LEVEL = 0x10

class STATUS_RETURN:
    NO_STATUS_PACKET = 0
    RETURN_ONLY_FOR_READ = 1
    RETRUN_FOR_ALL_PACKETS = 2

class ERROR_BIT_MASKS:
    INSTRUCTION = 0x40
    OVERLOAD = 0x20
    SEND_CHECKSUM = 0x10
    RANGE = 0x08
    OVERHEATING = 0x04
    ANGLE_LIMIT = 0x02
    INPUT_VOLTAGE = 0x01

# Controls which error conditions (see ALARM_BIT_MASKS) cause
# the LED to flash (indicating an error).
ALARM_LED = 0x11

# Controls which errors conditions (see ALARM_BIT_MASKS) cause
# the motors to shutdown.
ALARM_SHUTDOWN = 0x12
TORQUE_ENABLE = 0x18

# Controls the state of the LED (for the AX-18A, either ON or OFF)
LED = 0x19

class LED_STATE:        
    ON = 1
    OFF = 0

# The error between the goal position and the preset position.
CW_COMPLIANCE_MARGIN = 0x1A
CCW_COMPLIANCE_MARGIN = 0x1B

# Controls the flexibility of the servo; higher values result in more flexibility.
CW_COMPLIANCE_SLOPE = 0x1C
CCW_COMPLIANCE_SLOPE = 0x1D

# The target position of the servo.
GOAL_POSITION = 0x1E

# The maximum speed at which the servo can be moved.
MOVING_SPEED = 0x20

# The maximum torque.
TORQUE_LIMIT = 0x22

# The current position of the servo. 
PRESENT_POSITION = 0x24

# The current speed of the servo.
PRESENT_SPEED = 0x26

# The current load of the servo. 
PRESENT_LOAD = 0x28

# The current voltage as measured by the servo (units are 100mV). 
PRESENT_VOLTAGE = 0x2A

# The current temperature as measured by the servo (units are deg Celsius).
PRESENT_TEMPERATURE = 0x2B

# Set to 1 when a register is written, and 0 when an action packet is sent.
REGISTERED = 0x2C

# 0 if the servo has reached the goal position; 1 otherwise.
MOVING = 0x2E

# 0 if the EEPROM area can be modified; 1 otherwise.
# If the EEPROM is locked, the power to the servo must be reset to unlock it.
LOCK = 0x2F

# Unknown--documentation unlear.
PUNCH = 0x30

# The default values for registers which have default values.
DEFAULT_VALUES = {
    MODEL_NUMBER: 0x12,
    ID: 0x01,
    BAUD_RATE: 0x01,
    RETURN_DELAY: 0xFA,
    CW_ANGLE_LIMIT: 0x00,
    CCW_ANGLE_LIMIT: 0x3FF,
    TEMP_LIMIT: 0x46,
    MIN_VOLTAGE_LIMIT: 0x3C,
    MAX_VOLTAGE_LIMIT: 0x8C,
    MAX_TORQUE: 0x3D7,
    STATUS_RETURN_LEVEL: 0x02,
    ALARM_LED: 0x24,
    ALARM_SHUTDOWN: 0x24,
    TORQUE_ENABLE: 0x00,
    LED: 0x00,
    CW_COMPLIANCE_MARGIN: 0x01,
    CCW_COMPLIANCE_MARGIN: 0x01,
    REGISTERED: 0x00,
    MOVING: 0x00,
    LOCK: 0x00,
    PUNCH: 0x20
}