#!/usr/bin/env python3

# Copyright 2020 Embotech AG, Zurich, Switzerland
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

# Example code that demonstrates how the simulation package could be used for working
# with controllers. Simulation interfaces are created for the dynamics and a
# PI controller for the simple example of a pendulum. Then three simulations are run
# with different integral control gains, and a plot of the pendulum angle over time
# is shown for each gain choice for comparison.

import sys
import os
import minisim
import numpy as np
import matplotlib.pyplot as plt

# Add the package path to the python path temporarily. Useful to make the package
# testable anywhere and immediately, also without modifying the system python path.
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))


class PendulumState(minisim.SerdeInterface):
    def __init__(self, angle: float, angular_velocity: float):
        self.angle = angle
        self.angular_velocity = angular_velocity

    def serialize(self) -> np.ndarray:
        return np.array([self.angle, self.angular_velocity])

    @staticmethod
    def deserialize(serialized: np.ndarray) -> "PendulumState":
        return PendulumState(serialized[0], serialized[1])


class PendulumCommand(minisim.SerdeInterface):
    def __init__(self, torque: float):
        self.torque = torque

    def serialize(self) -> np.ndarray:
        return np.array([self.torque])

    @staticmethod
    def deserialize(serialized: np.ndarray) -> "PendulumState":
        return PendulumCommand(serialized[0])


class PendulumDynamics(minisim.DynamicsInterface):
    def __init__(self, length: float, damping: float):
        self.length = length
        self.damping = damping

    def evaluate_dynamics(
        self, current_state: PendulumState, current_command: PendulumCommand
    ) -> PendulumState:
        gravity = 9.81
        return PendulumState(
            angle=current_state.angular_velocity,
            angular_velocity=(
                -self.damping * current_state.angular_velocity
                - (gravity / self.length) * np.sin(current_state.angle)
                + current_command.torque
            ),
        )

    def evaluate_dynamics_serialized(
        self, current_state: np.ndarray, current_command: np.ndarray
    ) -> np.ndarray:
        return self.evaluate_dynamics(
            PendulumState.deserialize(current_state),
            PendulumCommand.deserialize(current_command),
        ).serialize()


class PendulumController(minisim.ControlInterface):
    def __init__(
        self, proportional_gain: float, integral_gain: float, target_angle: float
    ):
        self.proportional_gain = proportional_gain
        self.integral_gain = integral_gain
        self.target_angle = target_angle

        self.integrator_state = 0.0

    def reset(self):
        self.integrator_state = 0.0

    def compute_control(self, current_state: PendulumState) -> PendulumCommand:
        offset = self.target_angle - current_state.angle
        self.integrator_state += offset
        return PendulumCommand(
            torque=(
                (self.proportional_gain * offset)
                + (self.integral_gain * self.integrator_state)
            )
        )


def main():
    # First we create the objects that will be involved in the simulation:
    # - The dynamics, implementing minisim.DynamicsInterface
    dynamics = PendulumDynamics(length=2.0, damping=0.2)
    # - The controller, implementing minisim.ControlInterface
    controller = PendulumController(
        proportional_gain=2.5, integral_gain=0.0, target_angle=0.5
    )
    # - The initial state of the system
    initial_state = PendulumState(angle=0.4, angular_velocity=0.0)

    # We also create an object that will record the simulation history to memory
    # for later analysis. This object has to implement minisim.RecorderInterface.
    memory_recorder = minisim.SimulationRecorderToMemory()

    # Create a simulation with the configured objects
    step_time = 0.05  # seconds
    s = minisim.MiniSim(dynamics, step_time, listeners={"recorder": memory_recorder},)

    # Try out different controller gains
    controller_gains = [0.0, 0.01, 0.02]
    for gain in controller_gains:
        # The objects are stored in the simulator by reference, so we can
        # modify them here and have the changes stick in the simulation object
        memory_recorder.reset()
        controller.integral_gain = gain
        controller.reset()

        # Perform simulation and extract the history
        s.simulate_all(100.0, initial_state, controller)
        angles = list(map(lambda instant: instant.state.angle, memory_recorder.history))
        times = list(map(lambda instant: instant.time, memory_recorder.history))

        # Add a line to the plot
        plt.plot(times, angles)

    # Add some prettiness
    plt.legend([f"Integral gain {g}" for g in controller_gains])
    plt.xlabel("Time [s]")
    plt.ylabel("Pendulum angle [rad]")
    plt.grid(True)

    # Show the plot, this blocks until it is closed.
    plt.show()


if __name__ == "__main__":
    main()
