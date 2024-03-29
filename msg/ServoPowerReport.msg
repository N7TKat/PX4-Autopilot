uint64 timestamp                                        # time since system start (microseconds)


float32 servo_power_voltage     # Voltage V
float32 servo_power_current     # Current A
uint8 servo_power_error_flags   # 1= over voltage, 2= under voltage, 4= over current, 8= over current protection]

uint8 servo_power_address       #CANNode address

uint8  SERVO_FAILURE_OVER_VOLTAGE = 1
uint8  SERVO_FAILURE_UNDER_VOLTAGE = 2
uint8  SERVO_FAILURE_OVER_CURRENT = 4
uint8  SERVO_FAILURE_OVER_CURRENT_PROTECT = 8