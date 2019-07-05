# VehicleStateReport.msg

builtin_interfaces/Time stamp
uint8 fuel # 0 to 100
uint8 blinker
uint8 headlight
uint8 wiper
uint8 gear
uint8 mode
bool hand_brake
bool horn

### Definitions
# Blinker
uint8 BLINKER_OFF = 0
uint8 BLINKER_LEFT = 1
uint8 BLINKER_RIGHT = 2
uint8 BLINKER_HAZARD = 3
# Headlight
uint8 HEADLIGHT_OFF = 0
uint8 HEADLIGHT_ON = 1
uint8 HEADLIGHT_HIGH = 2
# Wiper
uint8 WIPER_OFF = 0
uint8 WIPER_LOW = 1
uint8 WIPER_HIGH = 2
uint8 WIPER_CLEAN = 3
# Gear
uint8 GEAR_DRIVE = 0
uint8 GEAR_REVERSE = 0
uint8 GEAR_PARK = 2
uint8 GEAR_LOW = 3
uint8 GEAR_NEUTRAL = 4
# Autonomous
uint8 MODE_MANUAL = 0
uint8 MODE_NOT_READY = 1
uint8 MODE_AUTONOMOUS = 2
uint8 MODE_DISENGAGED = 3
