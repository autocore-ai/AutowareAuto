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

# This module implements a small, but general simulator that interacts
# with application-specific code via some interfaces that are also defined here.

from abc import ABC, abstractmethod
import numpy as np
from typing import List

from scipy.integrate import odeint


# --- Define abstract interface classes for the simulation ---------------------------
class SerdeInterface(ABC):
    """
    Interface for objects implementing (de-)serialization to numpy ndarrays.

    The serialization can be used for purposes such as interfacing with integrators
    from numerical packages like scipy.integrate.ode.
    """

    @abstractmethod
    def serialize(self) -> np.ndarray:
        """Turn the object into a numpy ndarray."""

    @staticmethod
    @abstractmethod
    def deserialize(serialized_state: np.ndarray) -> "SerdeInterface":
        """Turn the given ndarray into an object. Inverse of serialize()."""


class DynamicsInterface(ABC):
    """
    Interface for objects implementing a continuous system dynamics function evaluation.

    That is a function of the form dx/dt = f(x,u),
    where x is the system state, u is the control input, and dx/dt is the
    derivative of the state for those arguments.
    The interface provides both structured and unstructured (as in, with
    serialized data) calls.
    """

    @abstractmethod
    def evaluate_dynamics(
        self, current_state: SerdeInterface, current_command: SerdeInterface
    ) -> SerdeInterface:
        """
        Return the derivative of current state.

        Given current_command is being applied to the system inputs.
        """

    @abstractmethod
    def evaluate_dynamics_serialized(
        self, current_state: np.ndarray, current_command: np.ndarray
    ) -> np.ndarray:
        """
        Return the derivative of current state.

        Given current_command is being applied to the system inputs.
        This can just be a wrapper call to evaluate_dynamics.
        """


class ControlInterface(ABC):
    @abstractmethod
    def compute_control(self, current_state: SerdeInterface) -> SerdeInterface:
        """
        Compute the control inputs that should be applied to the system.

        Given the state measurement current_state.
        """


class SimulationInstant:
    """Data class for storing a single time instant of the simulation."""

    def __init__(self, state, command, time):
        self.state = state
        self.command = command
        self.time = time


class RecorderInterface(ABC):
    @abstractmethod
    def record_instant(self, instant: SimulationInstant):
        """Record a SimulationInstant to the history."""


class SimulationRecorderToMemory(RecorderInterface):
    """Class to record simulation history to memory (stored within the object)."""

    def __init__(self):
        self.history: List[SimulationInstant] = []

    def reset(self):
        """Clear history."""
        self.history = []

    def record_instant(self, instant: SimulationInstant):
        """Add an instant to the history."""
        self.history += [instant]


# --- Main simulation class ----------------------------------------------------------
class MiniSim:
    """
    Simple and small one-object, one-controller simulator.

    See the documentation of the interfaces of the classes required by the constructor
    for how to implement dynamics and controller.
    """

    def __init__(
        self,
        dynamics: DynamicsInterface,
        step_time_seconds: float,
        listeners: dict = {},
    ):
        """
        Create a simulation object with the specified components.

        Listeners is a dictionary of objects for callback functionality. Currently
        supported keys:
          - "recorder": object implementing RecorderInterface. Used to either record
          or otherwise publish each simulated simulated state and control input.
        """
        self.dynamics = dynamics
        self.step_time = step_time_seconds
        self.listeners = listeners
        self.simulation_time = 0.0

    def _integrate_dynamics(
        self, state: SerdeInterface, command: SerdeInterface
    ) -> SerdeInterface:
        # Create a wrapper that closes over the current command
        def ode_wrapper(y: np.ndarray, t: float) -> np.ndarray:
            return self.dynamics.evaluate_dynamics_serialized(y, command.serialize())

        # Integrate the ODE using that facade
        solution_trajectory = odeint(
            ode_wrapper, state.serialize().flatten(), [0, self.step_time]
        )

        # Extract the final state from the output trajectory of the integrator
        integrated_end_state_serialized = solution_trajectory[-1, :]

        # Deserialize again, using the static function of the state class,
        # accessed through the input state object.
        return state.deserialize(integrated_end_state_serialized)

    def simulate_one_timestep(
        self,
        current_state: SerdeInterface,
        current_command: SerdeInterface,
        current_time: float,
    ) -> SerdeInterface:
        """
        Perform one iteration of the simulation.

        Takes the current state and command as an input and returns the next
        state. This function can be used for externally-controlled simulations,
        where the main loop is implemented as part of something else and the
        simulation object is just used in a re-entrant manner.

        This function is pure if the dynamics evaluation is also pure.
        """
        if "recorder" in self.listeners:
            self.listeners["recorder"].record_instant(
                SimulationInstant(current_state, current_command, current_time)
            )

        next_state = self._integrate_dynamics(current_state, current_command)
        return next_state

    def simulate_all(
        self,
        total_simulation_time,
        initial_state: SerdeInterface,
        controller: ControlInterface,
    ):
        """
        Perform a simulation for a given amount of time.

        This function is used as a "one stop shop" for doing entire simulations
        where the simulator can take control over the thread. This makes use
        of the same simulation_iteration function that is also individually
        accessible.
        """
        self.simulation_time = 0.0

        number_of_steps = int(np.floor(total_simulation_time / self.step_time))
        current_state = initial_state

        for k in range(number_of_steps):
            # Compute control and state update
            current_command = controller.compute_control(current_state)
            next_state = self.simulate_one_timestep(
                current_state, current_command, self.simulation_time
            )

            # Advance simulation state
            self.simulation_time += self.step_time
            current_state = next_state
