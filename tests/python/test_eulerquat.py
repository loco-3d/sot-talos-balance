from __future__ import print_function

import numpy as np
from dynamic_graph.sot_talos_balance.euler_to_quat import EulerToQuat
from dynamic_graph.sot_talos_balance.pose_quaternion_to_matrix_homo import PoseQuaternionToMatrixHomo
from dynamic_graph.sot_talos_balance.quat_to_euler import QuatToEuler
from numpy.testing import assert_almost_equal

# --- Euler to quat ---
print("--- Euler to quat ---")

signal_in = np.array([0.0, 0.0, 0.5, 0.0, 0.0, np.pi, 0.2, 0.6])
e2q = EulerToQuat('e2q')
e2q.euler.value = signal_in
print(e2q.euler.value)
e2q.quaternion.recompute(0)
print(e2q.quaternion.value)

assert_almost_equal(e2q.quaternion.value, np.array([0.0, 0.0, 0.5, 0.0, 0.0, 1.0, 0.0, 0.2, 0.6]), 6)

# --- Quat to Euler ---
print("--- Quat to Euler ---")

signal_in = np.array([0.0, 0.0, 0.5, 0.0, 0.0, 1.0, 0.0, 0.2, 0.6])
q2e = QuatToEuler('q2e')
q2e.quaternion.value = signal_in
print(q2e.quaternion.value)
q2e.euler.recompute(0)
print(q2e.euler.value)

assert_almost_equal(q2e.euler.value, np.array([0.0, 0.0, 0.5, 0.0, 0.0, np.pi, 0.2, 0.6]), 6)

# --- Quat to homogeneous ---
print("--- Quat to homogeneous ---")

signal_in = np.array([0.0, 0.0, 0.5, 0.0, 0.0, 1.0, 0.0])
q2m = PoseQuaternionToMatrixHomo('q2m')
q2m.sin.value = signal_in
print(q2m.sin.value)
q2m.sout.recompute(0)
print(q2m.sout.value)

expected = np.array(((-1.0, 0.0, 0.0, 0.0), (0.0, -1.0, 0.0, 0.0), (0.0, 0.0, 1.0, 0.5), (0.0, 0.0, 0.0, 1.0)))
assert_almost_equal(q2m.sout.value, expected, 6)
