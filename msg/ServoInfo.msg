uint64 timestamp                                 	# time since system start (microseconds)

uint16 counter                                          # incremented by the writing thread everytime new data is stored
uint8 count                                         # number of connected ESCs
uint8 connectiontype                                # how ESCs connected to the system

uint32[4] errorcount                                  # Number of reported errors by ESC - if supported

float32[4] temperature                                 # Temperature measured from current ESC [degC] - if supported

# Enumerator of SERVO_FAILURE_FLAGS
uint8 FAILURE_NONE = 0
uint8 FAILURE_OVER_CURRENT = 1
uint8 FAILURE_OVER_VOLTAGE = 2
uint8 FAILURE_MOTOR_OVER_TEMPERATURE = 4
uint8 FAILURE_OVER_RPM = 8
uint8 FAILURE_INCONSISTENT_CMD = 16
uint8 FAILURE_MOTOR_STUCK = 32
uint8 FAILURE_GENERIC = 64


uint8 CONNECTED_ESC_MAX = 8                             # The number of ESCs supported. Current (Q2/2013) we support 8 ESCs

# Enumerator of SERVO_CONNECTION_TYPE
uint8 ESC_CONNECTION_TYPE_PPM = 0                       # Traditional PPM ESC
uint8 ESC_CONNECTION_TYPE_SERIAL = 1                    # Serial Bus connected ESC
uint8 ESC_CONNECTION_TYPE_ONESHOT = 2                   # One Shot PPM
uint8 ESC_CONNECTION_TYPE_I2C = 3                       # I2C
uint8 ESC_CONNECTION_TYPE_CAN = 4                       # CAN-Bus
uint8 ESC_CONNECTION_TYPE_DSHOT = 5                     # DShot
