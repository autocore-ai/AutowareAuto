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

import motion_model_testing_simulator.minisim as minisim
import numpy as np


class BicycleState(minisim.SerdeInterface):
    """
    Class describing the states of a simple bicycle model.

    Described in https://www.researchgate.net/publication/318810853
    with an added state for the current steering angle (delta).
    """

    def __init__(self, x, y, v, phi):
        """
        Create a BicycleStates object with the given states.

        Parameters
        ----------
        x : float64
            The x position in the world frame.
        y : float64
            The y position in the world frame.
        v : float64
            The speed of the center of the bicycle (as described in the cited paper)
        phi : float64
            The heading of the bicycle (counter-clockwise, positive x is 0)
        delta : float64
            The steering angle of the front wheel (counter-clockwise, 0 means zero steering)

        """
        self.x = x
        self.y = y
        self.v = v
        self.phi = phi

    def serialize(self) -> np.ndarray:
        return np.array([self.x, self.y, self.v, self.phi])

    @staticmethod
    def deserialize(serialized: np.ndarray) -> "BicycleState":
        return BicycleState(*serialized)


class BicycleParameters:

    def __init__(self, cog_to_front, cog_to_rear, width):
        self.lf = cog_to_front
        self.lr = cog_to_rear
        self.width = width


class BicycleCommand(minisim.SerdeInterface):

    def __init__(self, acceleration, steering):
        self.acceleration = acceleration
        self.steering = steering

    def serialize(self) -> np.ndarray:
        return np.array([self.acceleration, self.steering])

    @staticmethod
    def deserialize(inputs_serialized) -> "BicycleCommand":
        return BicycleCommand(inputs_serialized[0], inputs_serialized[1])


class BicycleDynamics(minisim.DynamicsInterface):

    def __init__(self, parameters: BicycleParameters):
        self.parameters = parameters

    def evaluate_dynamics(
        self, states: BicycleState, u: BicycleCommand
    ) -> BicycleState:
        """
        Evaluate the continous-time dynamics function of the kinematic bicycle model.

        Given the current states and commands, return the derivative of
        the system state. The derivative is returned as a state object because
        every scalar state has a scalar derivative, and that way we can avoid
        introducing another type that is pretty much just a copy of
        BicycleState with renamed fields.
        """
        s = states
        lf, lr = self.parameters.lf, self.parameters.lr
        beta = np.arctan((lr * np.tan(u.steering)) / (lf + lr))
        return BicycleState(
            x=(s.v * np.cos(s.phi + beta)),
            y=(s.v * np.sin(s.phi + beta)),
            v=(u.acceleration),
            phi=(s.v / lr * np.sin(beta)),
        )

    def evaluate_dynamics_serialized(
        self, current_state: np.ndarray, current_command: np.ndarray
    ) -> np.ndarray:
        states = BicycleState.deserialize(current_state)
        inputs = BicycleCommand.deserialize(current_command)
        derivatives = self.evaluate_dynamics(states, inputs)
        return derivatives.serialize()
