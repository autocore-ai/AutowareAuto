#!/usr/bin/env python3

# Copyright 2020 StreetScooter GmbH, Aachen, Germany
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from rclpy.node import Node
from builtin_interfaces.msg import Duration
from builtin_interfaces.msg import Time

from autoware_auto_msgs.msg import Complex32
from autoware_auto_msgs.msg import Trajectory
from autoware_auto_msgs.msg import TrajectoryPoint
from autoware_auto_msgs.msg import VehicleKinematicState
from autoware_auto_msgs.msg import VehicleControlCommand

import math
import time
import sys

import matplotlib.pyplot as pp
import numpy as np

import motion_model_testing_simulator.minisim as minisim
import motion_model_testing_simulator.bicycle_model as bicycleModel


# TODO(s.me) this is probably available elsewhere...
def to_angle(heading: complex) -> float:
    # Translated from motion_common's "to_angle"
    magnitude = math.sqrt(heading.real ** 2 + heading.imag ** 2)
    if abs(magnitude - 1.0) > sys.float_info.epsilon:
        heading = complex(heading.real / magnitude, heading.imag / magnitude)

    y = 2.0 * heading.real * heading.imag
    x = 1.0 - (2.0 * heading.imag * heading.imag)
    return math.atan2(y, x)


# TODO(s.me) this is probably available elsewhere...
def from_angle(angle: float) -> complex:
    the_quaternion = Complex32()
    the_quaternion.real = math.cos(0.5 * angle)
    the_quaternion.imag = math.sin(0.5 * angle)
    return the_quaternion


class ControllerTestingNode(Node):
    """
    Node for testing the controller using a minisim simulation.

    This currently sends initial data to the controller node using the
    _start_test function, and then the simulation is advanced by the
    callback on receiving a command message (control_callback). This
    callback also sends a new message to the controller, keeping the
    loop going.
    """

    def __init__(self):
        super().__init__("controller_testing_node")
        self.declare_parameter("state_frame")
        self.declare_parameter("trajectory_frame")
        self.declare_parameter("sim_time_step_s")
        self.declare_parameter("vehicle.cog_to_front_axle")
        self.declare_parameter("vehicle.cog_to_rear_axle")

        # Publisher
        self._publisher_state = self.create_publisher(
            VehicleKinematicState, "vehicle_state", 0
        )
        self._publisher_trajectory = self.create_publisher(
            Trajectory, "planned_trajectory", 0
        )

        # Subscriber
        self._subscriber_controls = self.create_subscription(
            VehicleControlCommand, "control_command", self.control_callback, 0
        )

        # Timer to send initial trigger as soon as spinning
        self._timer_initial_trigger = self.create_timer(0.1, self._start_test)

        # Simulation and geometry parameters
        self.param_state_frame = self.get_parameter("state_frame")._value
        self.param_trajectory_frame = self.get_parameter("trajectory_frame")._value
        self.param_sim_time_step = self.get_parameter("sim_time_step_s")._value
        self.param_cog_to_front_axle = self.get_parameter(
            "vehicle.cog_to_front_axle"
        )._value
        self.param_cog_to_rear_axle = self.get_parameter(
            "vehicle.cog_to_rear_axle"
        )._value
        self._current_state = None

        # Init simulator
        # TODO(s.merkli) pick width also from ros parameters, sync with controller
        bicycle_parameters = bicycleModel.BicycleParameters(
            self.param_cog_to_front_axle, self.param_cog_to_rear_axle, width=2.0
        )
        vehicle_dynamics = bicycleModel.BicycleDynamics(bicycle_parameters)

        # We record the simulation states to memory for post-simulation evaluation
        self._memory_recorder = minisim.SimulationRecorderToMemory()

        # Create the simulation object. This does not simulate anything yet.
        self._simulator = minisim.MiniSim(
            vehicle_dynamics,
            self.param_sim_time_step,
            listeners={"recorder": self._memory_recorder},
        )

    def final_report(self):
        # TODO(s.merkli) expand on this - maybe also write the data to a file and analyze
        # in separate code.

        # Extract some of the relevant data
        def get_from_history(accessor):
            return list(map(lambda x: accessor(x), self._memory_recorder.history))

        x_history = get_from_history(lambda instant: instant.state.x)
        y_history = get_from_history(lambda instant: instant.state.y)
        time_history = get_from_history(lambda instant: instant.time)
        velocity_history = get_from_history(lambda instant: instant.state.v)
        cmd_a_history = get_from_history(lambda instant: instant.command.acceleration)
        cmd_steer_history = get_from_history(
            lambda instant: instant.command.steering_rate
        )
        steer_histroy = get_from_history(lambda instant: instant.state.delta)

        # Show an x-y plot of the CoG as well as a plot of velocity vs time
        pp.figure(0)
        pp.title("x-y history")
        pp.plot(x_history, y_history, "o")
        pp.plot(x_history, y_history, "o")
        pp.axis("equal")

        fig, ax = pp.subplots(2, 2, squeeze=True)
        fig.subplots_adjust(hspace=0.35)
        fig.subplots_adjust(wspace=0.4)
        fig.set_size_inches(10, 7)
        ax[0][0].set_title("time - acceleration command")
        ax[0][0].set(xlabel="simulation time [s]", ylabel="a cmd [m/s^2]")
        ax[0][0].scatter(time_history, cmd_a_history, marker="x", s=3.0, color="blue")
        ax[0][1].set_title("time - steering command")
        ax[0][1].set(xlabel="simulation time [s]", ylabel="wheel angle cmd [rad]")
        ax[0][1].scatter(
            time_history, cmd_steer_history, marker="x", s=3.0, color="blue"
        )
        ax[1][0].set_title("time - velocity state")
        ax[1][0].set(xlabel="simulation time [s]", ylabel="v state [m/s]")
        ax[1][0].scatter(
            time_history, velocity_history, marker="x", s=3.0, color="blue"
        )
        ax[1][1].set_title("time - steering state")
        ax[1][1].set(xlabel="simulation time [s]", ylabel="wheel angle state [rad]")
        ax[1][1].scatter(time_history, steer_histroy, marker="x", s=3.0, color="blue")

        # Show all figures in a blocking manner, once they're closed, exit.
        pp.show()

        # TODO(s.merkli) deal with the shutdown better: Looking for a signal_shutdown()
        # equivalent here? rclpy.shutdown() deadlocks This probably only
        # works, because this node is the only one running in spinner?
        self.destroy_node()

    def control_callback(self, current_command_msg):
        """Trigger simulator on reception of a new command from the controller."""
        # Check if we want to continue with the simulation
        if (
            self._simulator.simulation_time >= 10
        ):  # TODO(s.merkli) make stop time a parameter
            self.final_report()

        current_state = self.convert_vehicle_kinematic_to_bicycle_state(
            self._current_state
        )
        current_command = self.convert_to_bicycle_command(current_command_msg)

        # Trigger one simulation step and update current state. In externally
        # controlled simulations, we need to update the simulation time
        # ourselves as well.
        forward_simulated_state = self._simulator.simulate_one_timestep(
            current_state, current_command, self._simulator.simulation_time
        )
        self._simulator.simulation_time += self._simulator.step_time
        self._current_state = self.convert_bicycle_to_vehicle_kinematic_state(
            forward_simulated_state
        )

        # TODO(s.merkli) develop this further:
        # Check if we have to send another trajectory or just the current state?
        # Ignore the next callback if trajectory triggered?
        # Or keep it but somehow flagged?

        # Send mpc trigger again
        self._publisher_state.publish(self._current_state)

    def convert_bicycle_to_vehicle_kinematic_state(
        self, state: bicycleModel.BicycleState
    ) -> VehicleKinematicState:
        state_msg = VehicleKinematicState()
        # Transform odom_H_cog to odom_H_base_link
        # TODO(s.merkli): Double check
        state_msg.state.x = state.x - np.cos(state.phi) * self.param_cog_to_rear_axle
        state_msg.state.y = state.y - np.sin(state.phi) * self.param_cog_to_rear_axle

        state_msg.state.heading = from_angle(state.phi)
        state_msg.state.longitudinal_velocity_mps = state.v
        state_msg.state.lateral_velocity_mps = 0.0  # not modeled in this
        state_msg.state.acceleration_mps2 = 0.0  # modeled as an input
        state_msg.state.heading_rate_rps = 0.0  # modeled as an input
        state_msg.state.front_wheel_angle_rad = state.delta
        state_msg.state.rear_wheel_angle_rad = 0.0  # not modeled in this

        # Use simulated time
        state_msg.header.stamp = self.seconds_to_ros_time(
            self._simulator.simulation_time
        )
        # Use ROS time
        # state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.header.frame_id = self.param_state_frame
        return state_msg

    def convert_vehicle_kinematic_to_bicycle_state(
        self, state_msg: VehicleKinematicState
    ) -> bicycleModel.BicycleState:
        phi = to_angle(state_msg.state.heading)
        # Transform odom_H_base_link to odom_H_cog
        # TODO(s.merkli): Double check
        x_cog = float(state_msg.state.x + np.cos(phi) * self.param_cog_to_rear_axle)
        y_cog = float(state_msg.state.y + np.sin(phi) * self.param_cog_to_rear_axle)
        return bicycleModel.BicycleState(
            x=x_cog,
            y=y_cog,
            v=state_msg.state.longitudinal_velocity_mps,
            phi=phi,
            # TODO(s.merkli) this is currently wrong with no easy fix: The vehicle state
            # message provides the desired value of the angle, but reasonably
            # realistic dynamics should take as input the rate of change of the
            # angle instead.
            delta=state_msg.state.front_wheel_angle_rad,
        )

    def convert_to_bicycle_command(
        self, command_msg: VehicleControlCommand
    ) -> bicycleModel.BicycleCommand:
        # TODO(s.merkli) the second parameter should be the desired _derivative_ of
        # front wheel angle, not the front wheel angle itself.
        return bicycleModel.BicycleCommand(
            command_msg.long_accel_mps2, command_msg.front_wheel_angle_rad
        )

    def _start_test(self):
        # Only call this once - so destroy the calling timer
        self.destroy_timer(self._timer_initial_trigger)
        self._timer_initial_trigger = None

        # Initial system state
        init_state_msg = VehicleKinematicState()
        # Use simulated time
        init_state_msg.header.stamp = self.seconds_to_ros_time(
            self._simulator.simulation_time
        )
        # Use ROS time
        # init_state_msg.header.stamp = self.get_clock().now().to_msg()
        init_state_msg.header.frame_id = self.param_state_frame
        self._current_state = init_state_msg

        # Initial trajectory, starting at the current state
        # TODO(s.merkli): Potentially use a fancier spoofer later
        init_trajectory_msg = self.create_straight_line_trajectory(
            self._current_state, 10, 1.0, 0.1
        )

        # Use simulated time
        init_trajectory_msg.header.stamp = self.seconds_to_ros_time(
            self._simulator.simulation_time
        )
        # Use ROS time
        # init_trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        init_trajectory_msg.header.frame_id = self.param_trajectory_frame

        # Send first data to mpc
        # Both publishes trigger a control calculation but for the first,
        # there is missing information, so nothing happens
        self._publisher_trajectory.publish(init_trajectory_msg)
        # Make sure trajectory is sent first - TODO(s.merkli) can this be done
        # more systematically?
        time.sleep(0.1)
        self._publisher_state.publish(self._current_state)

    def seconds_to_ros_duration(self, seconds_in):
        secs = int(seconds_in)
        nsecs = int((seconds_in - secs) * math.pow(10, 9))
        return Duration(sec=secs, nanosec=nsecs)

    def seconds_to_ros_time(self, seconds_in):
        secs = int(seconds_in)
        nsecs = int((seconds_in - secs) * math.pow(10, 9))
        return Time(sec=secs, nanosec=nsecs)

    def create_straight_line_trajectory(
        self, init_state, length, speed, discretization_m
    ):
        num_points = int(length / discretization_m)
        num_points_max = 100  # max. length in Trajectory.msg
        if num_points > num_points_max:
            num_points = num_points_max
            self.get_logger().warn(
                "Only 100 points available - discretization set to %s"
                % float(length / num_points_max)
            )
        discretization_distance_m = float(length / num_points)
        discretization_time_s = float(discretization_distance_m / speed)

        trajectory_msg = Trajectory()

        # start at base_link
        init_point = init_state.state
        trajectory_msg.points.append(init_point)

        heading_angle = to_angle(init_point.heading)
        self.get_logger().warn(f"heading angle is {heading_angle}")

        for i in range(1, num_points - 1):
            trajectory_point = TrajectoryPoint()
            trajectory_point.time_from_start = self.seconds_to_ros_duration(
                discretization_time_s * i
            )

            trajectory_point.x = discretization_m * i * np.cos(heading_angle)
            trajectory_point.y = discretization_m * i * np.sin(heading_angle)
            trajectory_point.heading = init_point.heading
            trajectory_point.longitudinal_velocity_mps = float(speed)

            trajectory_msg.points.append(trajectory_point)

        return trajectory_msg
