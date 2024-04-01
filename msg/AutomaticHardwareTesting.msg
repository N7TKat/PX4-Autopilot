uint64 timestamp				# time since system start (microseconds)

# Topic to test individual mode

uint8 test_mode					# Test mode

uint8 TEST_MODE_RELEASE_ACTUATOR = 0		# Reserved for resetting
uint8 TEST_MODE_MOTOR_ONLY = 1  		# Mode No. Trigger 1st : Motor only Mode
uint8 TEST_MODE_SERVO_ONLY = 2			# Mode No. Trigger 2st : Servo only Mode
uint8 TEST_MODE_MOTOR_THEN_SERVO = 3		# Mode No. Trigger 3st : Motor then Servo Mode
uint8 TEST_MODE_STOP = 4			# Mode No. Trigger 4st : stop testing

uint16 each_function

bool inprogress

bool success

bool failed
