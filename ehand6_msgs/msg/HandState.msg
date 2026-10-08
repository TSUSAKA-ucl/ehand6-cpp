# State feedback of the eHand-6 (response to a read request).
std_msgs/Header header

# What triggered this publication: "poll" (periodic), "move" (monitoring after a
# position command) or "reset" (waiting for zeroing to finish)
string source

uint8 MSG_FORMAT_ERROR=0
uint8 MSG_ACTIVE_UPLOAD=1
uint8 MSG_READ_RESPONSE=2
uint8 MSG_WRITE_RESPONSE=3

uint8 msg_type
uint8 motor_mask       # 6-bit ID field of byte 1
uint8 system_state     # same codes as HandJointState.STATE_*
uint8 system_fault     # same codes as HandJointState.FAULT_*
bool all_standby       # true if every joint is in STANDBY

# order: thumb_h, thumb_v, index, middle, ring, little (motor No.1..6)
HandJointState[6] joints

uint8[32] raw          # raw 32-byte payload, for debugging
