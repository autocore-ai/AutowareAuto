# Copyright 2020 Embotech AG
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# This file contains tests for the record and replay behavior of the containing node.
# I'll start by conceptually documenting the tests I want to add, then implement them.


# TODO(s.me): Test: Check if "happy case" of recording, then replaying works
# - Start recordreplay_planner_node exe
# - Use ros2 commandline to send it a "start recording" action request
# - Use ros2 commandline to publish a few VehicleKinematicState messages
# - Cancel action by sending a signal SIGTERM to the ros2 commandline action process
# - Use ros2 commandline or python to listen on the specified trajectory topic, storing to memory
# - Use ros2 commandline or python to send it a "start replaying" action request
# - Use ros2 commandline or python to publish a few VehicleKinematicState messages
# - Cancel action by sending a signal to the ros2 commandline process
# - Verify that the replayed trajectories behaved as expected


# TODO(s.me): Test: Check if an additional record action is rejected if one is already running
# - Start recordreplay_planner_node exe
# - Use ros2 commandline to send it a "start recording" action request
# - Attempt to start a second start, verify this is rejected

# TODO(s.me): Test: Check if an additional replay action is rejected if one is already running
# - Start recordreplay_planner_node exe
# - Record a bit of trajectory like in happy case test
# - Use ros2 commandline to send it a "start replaying" action request
# - Attempt to start a second start, verify this is rejected

# TODO(s.me): Test: Check if replay stops when the trajectory being put out becomes empty.
# This is not implemented in the actual code yet - maybe not stopping but just giving out
# the last one-state trajectory is a better idea.
