uint64 timestamp                 # time since system start (microseconds)

float32 servo_position           # feedback position in ° [degree]
float32 servo_speed              # feedback speed in °deg/sec
float32 servo_force              # feedback of torque kg.cm
uint8 load_pct                   # Load percentage

uint8 servo_id                   # id of servo
uint8 servo_function             # servo output function (one of Servo1...ServoN)

uint8 servo_address              # CANNode Address