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
from rclpy.duration import Duration
from rclpy.time import Time

from autoware_auto_msgs.msg import Complex32
from autoware_auto_msgs.msg import ControlDiagnostic
from autoware_auto_msgs.msg import Trajectory
from autoware_auto_msgs.msg import TrajectoryPoint
from autoware_auto_msgs.msg import VehicleKinematicState
from autoware_auto_msgs.msg import VehicleControlCommand

from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry

import math
import sys

import matplotlib.pyplot as pp
import matplotlib.cm as cm
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

        # Simulation and geometry parameters
        self.param_state_frame = self.declare_parameter("state_frame").value
        self.param_odom_child_frame = self.declare_parameter("odom_child_frame").value
        self.param_sim_time_step = self.declare_parameter("sim_time_step_s").value
        self.param_stop_n_report_time_s = self.declare_parameter("stop_and_report_time_s").value
        self.param_real_time_sim = self.declare_parameter("real_time_sim", False).value
        self.param_cog_to_front_axle = self.declare_parameter(
            "vehicle.cog_to_front_axle"
        ).value
        self.param_cog_to_rear_axle = self.declare_parameter(
            "vehicle.cog_to_rear_axle"
        ).value
        self.param_wheelbase = self.param_cog_to_rear_axle + self.param_cog_to_front_axle
        self.param_trajectory_generate = self.declare_parameter("trajectory.generate").value
        self.param_trajectory_frame = self.declare_parameter("trajectory.frame").value
        self.param_trajectory_length = self.declare_parameter("trajectory.length").value
        self.param_trajectory_discretization_m = self.declare_parameter(
            "trajectory.discretization_m"
        ).value
        self.param_trajectory_speed_start = self.declare_parameter("trajectory.speed_start").value
        self.param_trajectory_speed_max = self.declare_parameter("trajectory.speed_max").value
        self.param_trajectory_speed_increments = self.declare_parameter(
            "trajectory.speed_increments"
        ).value
        self.param_trajectory_stopping_decel = self.declare_parameter(
            "trajectory.stopping_decel"
        ).value
        self.param_heading_rate = self.declare_parameter("trajectory.heading_rate").value
        self.param_heading_rate_max = self.declare_parameter("trajectory.heading_rate_max").value
        self.param_heading_rate_increments = self.declare_parameter(
            "trajectory.heading_rate_increments"
        ).value

        # Publisher
        self._publisher_state = self.create_publisher(
            VehicleKinematicState, "vehicle_state", 0
        )
        self._publisher_odometry = self.create_publisher(
            Odometry, 'odom', 10
        )
        self._publisher_tf = self.create_publisher(
            TFMessage, '/tf', 10
        )
        if self.param_trajectory_generate:
            self._publisher_trajectory = self.create_publisher(
                Trajectory, "planned_trajectory", 10
            )

        # Subscriber
        self._subscriber_controls = self.create_subscription(
            VehicleControlCommand, "control_command", self.control_callback, 0
        )

        if not self.param_trajectory_generate:
            self._subscriber_trajectory = self.create_subscription(
                Trajectory, "planned_trajectory", self.trajectory_callback, 0
            )

        self._subscriber_control_diag_ = self.create_subscription(
            ControlDiagnostic, "control_diagnostic", self.control_diag_callback, 0
        )
        # Initializing
        self._current_command = None
        self._traj_cache = None
        self._diag_msgs = []

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

        self._current_state = bicycleModel.BicycleState(
            x=self.param_cog_to_rear_axle,
            y=0.0,
            v=0.0,
            phi=0.0,
        )
        self._prev_state = None

        tf = self.convert_bicycle_to_transform(
            self._current_state, self.get_clock().now()
        )
        self._publisher_tf.publish(tf)

        # timer for sim_tick
        self.timer = self.create_timer(self._simulator.step_time, self.sim_tick)
        self.init_time = self.get_clock().now()

    def sim_tick(self):
        """Entrypoint to execute sim step on regular interval."""
        # get current time, to be use for publishing
        now = Time(seconds=self._simulator.simulation_time)  # Use simulated time
        if self.param_real_time_sim:
            now = self.get_clock().now()  # Use ROS time

        # Trigger first output, to start controller
        if self._current_command is None:
            # initial publish
            self.publish_state(now)
            if self.param_trajectory_generate:
                self.publish_trajectory(now)

        elif self.param_real_time_sim:      # Realtime im update step
            self.update_vehicle_state()
            self.publish_state(now)

        # incase controller is stuck, check for timeout and plot report
        if self.param_stop_n_report_time_s != 0 and not self.param_real_time_sim:
            # assuming non-realtime is at least 10 times faster, and adding 1 sec buffer
            assumed_timeout = (self.param_stop_n_report_time_s / 10.0) + 1.0
            if (self.get_clock().now() - self.init_time) > Duration(seconds=assumed_timeout):
                # only call in manual testing
                # self.final_report()
                self.shutdown()

    def control_callback(self, current_command_msg):
        """Store contol command and if faster than realtime, trigger simulator update."""
        self._current_command = self.convert_to_bicycle_command(current_command_msg)
        if not self.param_real_time_sim:
            self.update_vehicle_state()
            # TODO(s.merkli) develop this further:
            # Check if we have to send another trajectory or just the current state?
            # Ignore the next callback if trajectory triggered?
            # Or keep it but somehow flagged?

            # Send mpc trigger again
            self.publish_state(Time(seconds=self._simulator.simulation_time))

    def trajectory_callback(self, current_trajectory_msg):
        self._traj_cache = current_trajectory_msg

    def control_diag_callback(self, diag_msg):
        self._diag_msgs.append(diag_msg)

    def update_vehicle_state(self):
        """Update simulator state and publish."""
        # Check if we want to continue with the simulation
        if self.param_stop_n_report_time_s != 0:
            if self._simulator.simulation_time >= self.param_stop_n_report_time_s:
                # only call in manual testing
                # self.final_report()
                self.shutdown()

        # Trigger one simulation step and update current state. In externally
        # controlled simulations, we need to update the simulation time
        # ourselves as well.

        self._prev_state = self._current_state
        self._current_state = self._simulator.simulate_one_timestep(
            self._current_state, self._current_command, self._simulator.simulation_time
        )
        self._simulator.simulation_time += self._simulator.step_time

    def publish_state(self, now: Time):
        kinematic_state = self.convert_bicycle_to_vehicle_kinematic_state(
            self._current_state, now, self._prev_state, self._simulator.step_time
        )
        odom = self.convert_bicycle_to_odometry(
            self._current_state, now
        )
        tf = self.convert_bicycle_to_transform(
            self._current_state, now
        )

        self._publisher_state.publish(kinematic_state)
        self._publisher_odometry.publish(odom)
        self._publisher_tf.publish(tf)

    def publish_trajectory(self, now: Time):
        kinematic_state = self.convert_bicycle_to_vehicle_kinematic_state(
            self._current_state, now, self._prev_state, self._simulator.step_time
        )
        # Initial trajectory, starting at the current state
        # TODO(s.merkli): Potentially use a fancier spoofer later
        init_trajectory_msg = self.create_curved_trajectory(
            kinematic_state,
            self.param_trajectory_length,
            self.param_trajectory_discretization_m,
            self.param_trajectory_speed_start,
            self.param_trajectory_speed_max,
            self.param_trajectory_speed_increments,
            self.param_trajectory_stopping_decel,
            self.param_heading_rate,
            self.param_heading_rate_max,
            self.param_heading_rate_increments,
        )

        # Use simulated time
        init_trajectory_msg.header.stamp = now.to_msg()
        # Use ROS time
        # init_trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        init_trajectory_msg.header.frame_id = self.param_trajectory_frame

        # Send first data to mpc
        # Both publishes trigger a control calculation but for the first,
        # there is missing information, so nothing happens
        self._publisher_trajectory.publish(init_trajectory_msg)
        # Make sure trajectory is sent first - TODO(s.merkli) can this be done
        # more systematically?
        self._traj_cache = init_trajectory_msg

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
            lambda instant: instant.command.steering
        )

        # Show an x-y plot of the CoG as well as a plot of velocity vs time
        pp.figure(0)
        colors = cm.rainbow(np.linspace(0, 1, len(self._memory_recorder.history)))
        pp.title("x-y history")
        pp.scatter(x_history, y_history, color=colors)
        pp.axis("equal")

        fig, ax = pp.subplots(2, 2, squeeze=True)
        fig.subplots_adjust(hspace=0.35)
        fig.subplots_adjust(wspace=0.4)
        fig.set_size_inches(10, 7)
        ax[0][0].set_title("time - acceleration command")
        ax[0][0].set(xlabel="simulation time [s]", ylabel="a cmd [m/s^2]")
        ax[0][0].plot(time_history, cmd_a_history, marker=".", c="blue")
        ax[0][1].set_title("time - steering command")
        ax[0][1].set(xlabel="simulation time [s]", ylabel="wheel angle cmd [rad]")
        ax[0][1].plot(time_history, cmd_steer_history, marker=".", c="b")
        ax[1][0].set_title("time - velocity state")
        ax[1][0].set(xlabel="simulation time [s]", ylabel="v state [m/s]")
        ax[1][0].plot(time_history, velocity_history, marker=".", c="b", label='state')
        ax[1][1].set_title("time - steering state")
        ax[1][1].set(xlabel="simulation time [s]", ylabel="wheel angle state [rad]")

        # Add Trajectgory velocity to "time - velocity state" chart for comparison
        if self._traj_cache is not None:
            def get_from_traj_history(accessor):
                return list(map(lambda x: accessor(x), self._traj_cache.points))
            traj_time_history = get_from_traj_history(
                lambda instant: Duration.from_msg(instant.time_from_start).nanoseconds / 1e9
            )
            traj_velocity_history = get_from_traj_history(
                lambda instant: instant.longitudinal_velocity_mps
            )
            ax[1][0].plot(
                traj_time_history, traj_velocity_history, marker=".", c='r', label='traj'
            )
            ax[1][0].legend()
            ax[1][0].set_title("time - velocity")
            ax[1][0].set(xlabel="simulation time [s]", ylabel="v [m/s]")

        # Controller Diagnostics
        if len(self._diag_msgs) > 0:
            diag_init_time = Time.from_msg(self._diag_msgs[0].header.data_stamp).nanoseconds / 1e9

            def get_from_diag(accessor):
                return list(map(lambda x: accessor(x), self._diag_msgs))

            diag_time_history = get_from_diag(
                lambda instant:
                    (Time.from_msg(instant.header.data_stamp).nanoseconds / 1e9) - diag_init_time
            )
            lateral_error_m = get_from_diag(lambda instant: instant.lateral_error_m)
            longitudinal_error_m = get_from_diag(lambda instant: instant.longitudinal_error_m)
            velocity_error_mps = get_from_diag(lambda instant: instant.velocity_error_mps)
            acceleration_error_mps2 = get_from_diag(
                lambda instant: instant.acceleration_error_mps2
            )
            yaw_error_rad = get_from_diag(lambda instant: instant.yaw_error_rad)
            yaw_rate_error_rps = get_from_diag(lambda instant: instant.yaw_rate_error_rps)

            fig2, ax2 = pp.subplots(3, 2, squeeze=True)
            fig2.subplots_adjust(hspace=0.55)
            fig2.subplots_adjust(wspace=0.4)
            fig2.set_size_inches(10, 7)
            ax2[0][0].set_title("time - acceleration error")
            ax2[0][0].set(xlabel="simulation time [s]", ylabel="a cmd [m/s^2]")
            ax2[0][0].plot(diag_time_history, acceleration_error_mps2, marker=".", c="b")
            ax2[0][1].set_title("time - yaw rate error")
            ax2[0][1].set(xlabel="simulation time [s]", ylabel="yaw_rate_error [rad/s]")
            ax2[0][1].plot(diag_time_history, yaw_rate_error_rps, marker=".", c="b")
            ax2[1][0].set_title("time - velocity error")
            ax2[1][0].set(xlabel="simulation time [s]", ylabel="v error [m/s]")
            ax2[1][0].plot(diag_time_history, velocity_error_mps, marker=".", c="b", label='state')
            ax2[1][1].set_title("time - yaw error")
            ax2[1][1].set(xlabel="simulation time [s]", ylabel="yaw_error [rad]")
            ax2[1][1].plot(diag_time_history, yaw_error_rad, marker=".", c="b")

            ax2[2][0].set_title("time - lateral_error")
            ax2[2][0].set(xlabel="simulation time [s]", ylabel="lateral error[m]")
            ax2[2][0].plot(diag_time_history, lateral_error_m, marker=".", c="b", label='state')
            ax2[2][1].set_title("time - longitudinal error")
            ax2[2][1].set(xlabel="simulation time [s]", ylabel="longitudinal error [m]")
            ax2[2][1].plot(diag_time_history, longitudinal_error_m, marker=".", c="b")

        # Show all figures in a blocking manner, once they're closed, exit.
        pp.show()

    def shutdown(self):
        # TODO(s.merkli) deal with the shutdown better: Looking for a signal_shutdown()
        # equivalent here? rclpy.shutdown() deadlocks This probably only
        # works, because this node is the only one running in spinner?
        self.destroy_node()

    def convert_bicycle_to_vehicle_kinematic_state(
        self, state: bicycleModel.BicycleState, now: Time,
        prev_state: bicycleModel.BicycleState, dt_sec: float
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
        if (self._current_command is None):
            state_msg.state.front_wheel_angle_rad = 0.0
        else:
            state_msg.state.front_wheel_angle_rad = self._current_command.steering

        state_msg.state.rear_wheel_angle_rad = 0.0  # not modeled in this

        state_msg.header.stamp = now.to_msg()
        state_msg.header.frame_id = self.param_state_frame

        if prev_state is not None:
            # for rear-wheel center
            state_msg.state.heading_rate_rps = state.v * math.tan(self._current_command.steering) \
                / self.param_cog_to_rear_axle
            state_msg.state.acceleration_mps2 = (state.v - prev_state.v) / dt_sec

        return state_msg

    def convert_bicycle_to_odometry(
        self, state: bicycleModel.BicycleState, now: Time
    ) -> Odometry:
        odom_msg = Odometry()
        odom_msg.header.frame_id = self.param_state_frame
        odom_msg.child_frame_id = self.param_odom_child_frame
        odom_msg.header.stamp = now.to_msg()

        odom_msg.pose.pose.position.x = state.x - np.cos(state.phi) * self.param_cog_to_rear_axle
        odom_msg.pose.pose.position.y = state.y - np.sin(state.phi) * self.param_cog_to_rear_axle
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(state.phi / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(state.phi / 2.0)

        odom_msg.twist.twist.linear .x = state.v
        odom_msg.twist.twist.linear .y = 0.0
        odom_msg.twist.twist.linear .z = 0.0

        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        if (self._current_command is None):
            odom_msg.twist.twist.angular.z = 0.0
        else:
            odom_msg.twist.twist.angular.z = state.v \
                * math.tan(self._current_command.steering) / self.param_wheelbase

        return odom_msg

    def convert_bicycle_to_transform(
        self, state: bicycleModel.BicycleState, now: Time
    ) -> TFMessage:
        transform = TransformStamped()
        transform.header.frame_id = self.param_state_frame
        transform.header.stamp = now.to_msg()
        transform.child_frame_id = self.param_odom_child_frame

        transform.transform.translation.x = \
            state.x - np.cos(state.phi) * self.param_cog_to_rear_axle
        transform.transform.translation.y = \
            state.y - np.sin(state.phi) * self.param_cog_to_rear_axle
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(state.phi / 2.0)
        transform.transform.rotation.w = math.cos(state.phi / 2.0)

        return TFMessage(transforms=[transform])

    def convert_to_bicycle_command(
        self, command_msg: VehicleControlCommand
    ) -> bicycleModel.BicycleCommand:
        # TODO(s.merkli) the second parameter should be the desired _derivative_ of
        # front wheel angle, not the front wheel angle itself.
        return bicycleModel.BicycleCommand(
            command_msg.long_accel_mps2, command_msg.front_wheel_angle_rad
        )

    def create_curved_trajectory(
        self, init_state, length, discretization_m,
        speed_start, speed_max, speed_increments, stopping_decel,
        heading_rate, heading_rate_max, heading_rate_increments
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
        trajectory_msg = Trajectory()

        # start at base_link
        init_point = init_state.state
        trajectory_msg.points.append(init_point)

        stopping = False
        speed = speed_start
        seconds = float(discretization_distance_m / speed)

        cur_x = init_point.x
        cur_y = init_point.y
        heading_angle = to_angle(init_point.heading)
        prev_heading_angle = heading_angle
        prev_speed = speed

        for i in range(1, num_points - 1):
            # update speed profile
            if not stopping:
                speed += speed_increments
                stopping_time = speed / stopping_decel
                stopping_distance = \
                    speed * stopping_time \
                    - 0.5 * stopping_decel * stopping_time * stopping_time
                if ((num_points - i) * discretization_distance_m) <= stopping_distance:
                    stopping = True

            speed = min(speed, speed_max)

            if i == (num_points - 2):
                speed = 0.0

            if speed > 0:
                seconds_delta = float(discretization_distance_m / speed)
                seconds += seconds_delta
                if stopping:
                    speed -= stopping_decel * seconds_delta
                    speed = max(0.0, speed)

            # update heading
            heading_angle += heading_rate
            heading_rate += heading_rate_increments
            heading_rate = max(-heading_rate_max, min(heading_rate_max, heading_rate))

            # fillup trajectory point
            trajectory_point = TrajectoryPoint()
            trajectory_point.time_from_start = Duration(
                seconds=seconds
            ).to_msg()

            cur_x += discretization_m * np.cos(heading_angle)
            cur_y += discretization_m * np.sin(heading_angle)

            trajectory_point.x = cur_x
            trajectory_point.y = cur_y
            trajectory_point.heading = from_angle(heading_angle)
            trajectory_point.longitudinal_velocity_mps = float(speed)
            trajectory_point.acceleration_mps2 = (speed - prev_speed) / seconds
            trajectory_point.heading_rate_rps = (heading_angle - prev_heading_angle) / seconds

            trajectory_msg.points.append(trajectory_point)
            prev_heading_angle = heading_angle
            prev_speed = speed

        return trajectory_msg
