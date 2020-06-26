# Copyright 2018 Apex.AI, Inc.
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

# This file compute derivatives for constant acceleration turn rate motion model
# If you're having trouble installing sympy, try
#      pip install --user sympy

from sympy import symbols, cos, sin, integrate, diff, simplify
import pprint

v, a, th, w, T, t = symbols('v a th w T t')

print("{0} = velocity".format(v))
print("{0} = acceleration".format(a))
print("{0} = theta (heading, +x when 0)".format(th))
print("{0} = omega (turn rate, ccw positive)".format(w))
print("{0} = Time horizon (i.e. t in x(t))".format(T))

tht = th + w * t
vt = v + a * t

# define position derivatives
dxdot = vt * cos(tht)
dydot = vt * sin(tht)

# compute position as a function of time
dx = integrate(dxdot, (t, 0, T))
dy = integrate(dydot, (t, 0, T))

# compute partial derivatives
dxdv = diff(dx, v)
dxda = diff(dx, a)
dxdth = diff(dx, th)
dxdw = diff(dx, w)

dydv = diff(dy, v)
dyda = diff(dy, a)
dydth = diff(dy, th)
dydw = diff(dy, w)

# print
print("\n\n\nx:")
pprint(simplify(dx))

print("\n\ndx/dv = ")
pprint(simplify(dxdv))
print("\ndx/da = ")
pprint(simplify(dxda))
print("\ndx/dth = ")
pprint(simplify(dxdth))
print("\ndx/dw = ")
pprint(simplify(dxdw))

print("\n\n\ny:")
pprint(simplify(dy))

print("\n\ndy/dv = ")
pprint(simplify(dydv))
print("\ndy/da = ")
pprint(simplify(dyda))
print("\ndy/dth = ")
pprint(simplify(dydth))
print("\ndy/dw = ")
pprint(simplify(dydw))
