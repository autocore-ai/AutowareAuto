ne_raptor_interface {#ne_raptor_interface-package-design}
===========

This is the design document for the `ne_raptor_interface` package.


# Purpose / Use cases
This package acts as a message translator between New Eagle's Raptor DBW software and Autoware.Auto.


# Design
This package inherits from @ref vehicle-interface-design and, as such, performs all of the same tasks but specifically with messages from the `raptor_dbw_msgs` stack.


## Assumptions / Known limits
Current version of New Eagle's Raptor DBW Node is intended for R&D only!

## Inputs / Outputs / API

### Inputs
- Autoware Command messages are sent when the commanded values should change
- Raptor DBW Command messages are sent periodically (every 20-100ms advised)

#### From Autoware
- autoware_auto_msgs::msg::HighLevelControlCommand
- autoware_auto_msgs::msg::VehicleControlCommand
- autoware_auto_msgs::msg::VehicleStateCommand

#### To Raptor
- raptor_dbw_msgs::msg::AcceleratorPedalCmd
- raptor_dbw_msgs::msg::BrakeCmd
- raptor_dbw_msgs::msg::GearCmd
- raptor_dbw_msgs::msg::GlobalEnableCmd
- raptor_dbw_msgs::msg::MiscCmd
- raptor_dbw_msgs::msg::SteeringCmd

### Outputs
- Raptor DBW Report messages are received periodically
- Autoware Report messages are passed on once all relevant data has been compiled from the Raptor DBW Reports

#### To Autoware
- autoware_auto_msgs::msg::VehicleOdometry
- autoware_auto_msgs::msg::VehicleStateReport
- autoware_auto_msgs::msg::VehicleKinematicState

#### From Raptor
- raptor_dbw_msgs::msg::BrakeReport
- raptor_dbw_msgs::msg::GearReport
- raptor_dbw_msgs::msg::MiscReport
- raptor_dbw_msgs::msg::OtherActuatorsReport
- raptor_dbw_msgs::msg::SteeringReport
- raptor_dbw_msgs::msg::WheelSpeedReport

### Parameters
- `ecu_build_num` ECU build #
- `front_axle_to_cog` Distance from the front axle to the center-of-gravity of the vehicle in meters.
- `rear_axle_to_cog` Distance from the rear axle to the center-of-gravity of the vehicle in meters.
- `steer_to_tire_ratio` Ratio of steering angle / car tire angle
- `max_steer_angle` Maximum steering wheel turn angle
- `acceleration_limit` m/s^2, zero = no limit
- `deceleration_limit` m/s^2, zero = no limit
- `acceleration_positive_jerk_limit` m/s^3
- `deceleration_negative_jerk_limit` m/s^3
- `pub_period` message publishing period, in milliseconds

## Inner-workings / Algorithms
- Autoware Command messages are sent on change, while Raptor DBW Command messages must be sent periodically.

## Error detection and handling
- Catches invalid autonomy mode change requests.
- Catches invalid changes in speed
  - Cannot go forward while in reverse gear/go backwards while in a forward gear. Defaults to commanding speed = 0 in such cases.
- Constrains wheel angle commands to a valid range
- Catches invalid gear, wiper, headlight, & turn signal state requests.
- Catches contradicting wheel speeds

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
- [New Eagle: Platforms for Controlling Autonomous Vehicles](https://neweagle.net/autonomous-machines/)


# Future extensions / Unimplemented parts
- `autoware_auto_msgs::msg::RawControlCommand` not implemented - command units are undefined


# Related issues
- [#822](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/822)
- [#991](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/991)
- [#1025](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/1025)
