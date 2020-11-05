ssc_interface design {#ssc_interface-package-design}
====================


# Purpose / Use cases
This package acts as a message translator between AutonomouStuff's SSC software and Autoware.Auto.


# Design
This package inherits from @ref vehicle-interface-design and, as such, performs all of the same tasks but specifically with messages from the `automotive_autonomy_msgs` stack.


## Assumptions / Known limits
<!-- Required -->

## Inputs / Outputs / API

### Inputs

#### From Autoware
- autoware_auto_msgs::msg::VehicleControlCommand to command speed and steering
- autoware_auto_msgs::msg::VehicleStateCommand to control drive-by-wire mode, gear, and turn signals

#### From SSC
- automotive_platform_msgs::msg::GearFeedback
- automotive_platform_msgs::msg::SteeringFeedback
- automotive_platform_msgs::msg::VelocityAccelCov

### Outputs

#### To Autoware
- autoware_auto_msgs::msg::VehicleOdometry
- autoware_auto_msgs::msg::VehicleStateReport

#### To SSC
- automotive_platform_msgs::msg::GearCommand
- automotive_platform_msgs::msg::SpeedMode
- automotive_platform_msgs::msg::SteerMode
- automotive_platform_msgs::msg::TurnSignalCommand

### Parameters
- `front_axle_to_cog` Distance from the front axle to the center-of-gravity of the vehicle in meters.
- `rear_axle_to_cog` Distance from the rear axle to the center-of-gravity of the vehicle in meters.


## Inner-workings / Algorithms
`ssc_interface` employs an internal state machine to conform to some safety features in the SSC software and to avoid sending unintended control commands with an enable flag set to true.
The specific safety feature that is problematic is the requirement that a "disable" message be sent to each system before an "enable" message will be accepted.
The DbwStateMachine class internally holds a state which is dependent upon several factors and defaults to DbwState::DISABLED to avoid accidentally sending enabled control commands to SSC.
The state transition diagram for DbwStateMachine looks like this:

- Begin in DbwState::DISABLED
- If the user requests SSC to be enabled, transition to DbwState::ENABLE_REQUESTED
- Once an initial control command and an initial state command have been sent with the enabled flag set to false, start returning `true` in DbwStateMachine::enabled()
- Once a state command or control command with an enable flag set to true has been sent, transition to DbwState::ENABLE_SENT
  - While in state DbwState::ENABLE_SENT, debounce the DBW enabled status value returned by SSC to avoid disabling while DBW system is transitioning to enabled
  - If DBW system reports disabled multiple times (> debounce count), enabling failed and transition back to DbwState::DISABLED
- Once SSC reports that the DBW system has been enabled, transition to DbwState::ENABLED
- While in DbwState::ENABLED, immediately transition to DbwState::DISABLED if either the user requests it or the SSC reports it from the DBW system


## Error detection and handling
`ssc_interface` currently detects invalid gear and turn signal commands as well as requiring parameters to employ the SafetyStateMachine from `vehicle_interface`.


# Security considerations
<!-- Required -->
<!-- Things to consider:
- Spoofing (How do you check for and handle fake input?)
- Tampering (How do you check for and handle tampered input?)
- Repudiation (How are you affected by the actions of external actors?).
- Information Disclosure (Can data leak?).
- Denial of Service (How do you handle spamming?).
- Elevation of Privilege (Do you need to change permission levels during execution?) -->


# References / External links
- [Speed and Steering Control Software](https://autonomoustuff.com/product/astuff-speed-steering-control-software/)

# Future extensions / Unimplemented parts
<!-- Optional -->


# Related issues
- [#184](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/184)
