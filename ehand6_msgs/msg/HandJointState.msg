# One joint (motor) of the eHand-6. position / velocity are normalized: raw(0..255)/255.
uint8 STATE_INIT=0
uint8 STATE_STANDBY=1
uint8 STATE_CALIBRATING=2
uint8 STATE_POSITION_MODE=3
uint8 STATE_RESERVED4=4
uint8 STATE_AGING=5
uint8 STATE_FAULT=6
uint8 STATE_WAIT_RESPONSE=7

uint8 FAULT_NONE=0
uint8 FAULT_OVERCURRENT=1
uint8 FAULT_OVERVOLTAGE=2
uint8 FAULT_UNDERVOLTAGE=3
uint8 FAULT_OVERHEAT=4
uint8 FAULT_STALL=5
uint8 FAULT_COMM_TIMEOUT=6
uint8 FAULT_HARDWARE=7

string name
uint8 state
uint8 fault
float32 position   # 0..1
float32 velocity   # 0..1
