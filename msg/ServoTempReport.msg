uint64 timestamp                            # time since system start (microseconds)

float32 servo_temp_temperature              # temperture in °C

uint8 servo_temp_address                    # CANNode Address

uint8 servo_temp_error_flags                # 1: over heating, 2: over cooling

uint8  SERVO_FAILURE_OVER_HEATING = 1
uint8  SERVO_FAILURE_OVER_COOLING = 2
