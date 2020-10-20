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


class AckermannState(minisim.SerdeInterface):

    def __init__(self):
        pass

    def serialize(self) -> np.ndarray:
        return np.array([])

    @staticmethod
    def deserialize(serialized: np.ndarray) -> "AckermannState":
        return AckermannState()


class AckermannCommand(minisim.SerdeInterface):

    def __init__(self, acc_mps2: float, wheel_angle_rad: float):
        self.long_acc_mps2 = acc_mps2
        self.wheel_angle_rad = wheel_angle_rad

    def serialize(self) -> np.ndarray:
        return np.array([self.long_acc_mps2, self.wheel_angle_rad])

    @staticmethod
    def deserialize(serialized: np.ndarray) -> "AckermannCommand":
        return AckermannCommand(serialized[0], serialized[1])


class AckermannDynamics(minisim.DynamicsInterface):

    def __init__(self):
        pass

    def evaluate_dynamics(
        self, current_state: AckermannState, current_command: AckermannCommand
    ) -> AckermannState:
        return AckermannState()

    def evaluate_dynamics_serialized(
        self, current_state: np.ndarray, current_command: np.ndarray
    ) -> np.ndarray:
        return self.evaluate_dynamics(
            AckermannState.deserialize(current_state),
            AckermannCommand.deserialize(current_command),
        ).serialize()
