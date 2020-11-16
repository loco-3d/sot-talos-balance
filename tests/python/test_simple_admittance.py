from __future__ import print_function

import numpy as np
from dynamic_graph.sot_talos_balance.simple_admittance_controller import SimpleAdmittanceController
from numpy.testing import assert_almost_equal

controller = SimpleAdmittanceController("ciao")

print("\nSignals (at creation):")
controller.displaySignals()

N_JOINTS = 2

Kp = np.array(N_JOINTS * [10.0])
state = np.array(N_JOINTS * [0.0])
tauDes = np.array(N_JOINTS * [0.0])
tau = np.array(N_JOINTS * [0.0])

controller.Kp.value = Kp
controller.state.value = state
controller.tauDes.value = tauDes
controller.tau.value = tau

print("\nKp:   %s" % (controller.Kp.value, ))
print("\nq:    %s" % (controller.state.value, ))
print("tauDes: %s" % (controller.tauDes.value, ))
print("tau:    %s" % (controller.tau.value, ))

q = np.array(N_JOINTS * [1.0])
dt = 1

controller.init(dt, N_JOINTS)

controller.setPosition(q)
controller.qRef.recompute(0)

print("\nqRef: %s" % (controller.qRef.value, ))
assert_almost_equal(controller.qRef.value, q)

tauDes = np.array(N_JOINTS * [1.0])
controller.tauDes.value = tauDes

controller.qRef.recompute(1)

print("\nqRef: %s" % (controller.qRef.value, ))
qRef = np.array([q[i] + Kp[i] * (tauDes[i] - tau[i]) * dt for i in range(len(q))])
assert_almost_equal(controller.qRef.value, qRef)
