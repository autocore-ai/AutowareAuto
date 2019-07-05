# VehicleStateCommand.msg

builtin_interfaces/Time stamp
uint8 blinker 0
uint8 headlight 0
uint8 wiper 0
uint8 gear 0
uint8 mode 0
bool hand_brake false
bool horn false
bool autonomous false

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
