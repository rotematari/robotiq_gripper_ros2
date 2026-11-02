# Status of one Robotiq gripper, published by robotiq_gripper_driver.

std_msgs/Header header

string model            # hande, 2f85 or 2f140
bool connected          # false after a serial read failure, until a read succeeds again

# Activation
bool activated          # gACT == 1 and activation completed (gSTA == 3)
uint8 STATE_RESET = 0
uint8 STATE_ACTIVATING = 1
uint8 STATE_ACTIVATED = 3
uint8 activation_state  # gSTA

# Motion and object detection (gOBJ; valid only while go-to is active, gGTO == 1)
bool go_to_active       # gGTO
uint8 OBJECT_MOVING = 0
uint8 OBJECT_CONTACT_OPENING = 1
uint8 OBJECT_CONTACT_CLOSING = 2
uint8 OBJECT_AT_REQUEST = 3
uint8 object_status
bool moving             # go-to active and gOBJ == OBJECT_MOVING
bool object_detected    # go-to active and gOBJ is a contact state

# Faults
uint8 fault_code        # gFLT; 0 means no fault
string fault_message    # empty when there is no fault

# Physical values (approximate; see docs/design.md)
float64 width           # finger opening, m
float64 requested_width # opening for the echoed position request, m
float64 current         # motor current, A

# Raw registers
uint8 position_register
uint8 requested_register
uint8 current_register

# Model ranges, so clients can validate goals
float64 stroke          # m
float64 min_force       # N
float64 max_force       # N
float64 min_speed       # m/s
float64 max_speed       # m/s
