from __future__ import print_function

import numpy as np
from dynamic_graph.sot_talos_balance.com_admittance_controller import ComAdmittanceController
from numpy.testing import assert_almost_equal

controller = ComAdmittanceController("ciao")

print("\nSignals (at creation):")
controller.displaySignals()

Kp = np.array([10.0, 10.0, 0.0])
ddcomDes = np.array(3 * [0.0])
zmpDes = np.array(3 * [0.0])
zmp = np.array(3 * [0.0])

controller.Kp.value = Kp
controller.ddcomDes.value = ddcomDes
controller.zmpDes.value = zmpDes
controller.zmp.value = zmp

print()
print("Kp:       %s" % (controller.Kp.value, ))
print("ddcomDes: %s" % (controller.ddcomDes.value, ))
print("zmpDes:   %s" % (controller.zmpDes.value, ))
print("zmp:      %s" % (controller.zmp.value, ))

com = np.array(3 * [0.0])
dcom = np.array(3 * [0.0])
dt = 1

controller.init(dt)

controller.setState(com, dcom)
controller.comRef.recompute(0)
controller.dcomRef.recompute(0)

print()
print("comRef:  %s" % (controller.comRef.value, ))
assert_almost_equal(controller.comRef.value, com)
print("dcomRef: %s" % (controller.dcomRef.value, ))
assert_almost_equal(controller.dcomRef.value, dcom)

ddcomDes = np.array(3 * [1.0])
controller.ddcomDes.value = ddcomDes

print()
print("ddcomDes: %s" % (controller.ddcomDes.value, ))

controller.dcomRef.recompute(1)
controller.comRef.recompute(1)

print()

print("ddcomRef: %s" % (controller.ddcomRef.value, ))
ddcomRef = np.array([ddcomDes[i] + Kp[i] * (zmp[i] - zmpDes[i]) for i in range(3)])
assert_almost_equal(controller.ddcomRef.value, ddcomRef)

print("dcomRef:  %s" % (controller.dcomRef.value, ))
dcomRef = np.array([dcom[i] + ddcomRef[i] * dt for i in range(3)])
assert_almost_equal(controller.dcomRef.value, dcomRef)

print("comRef:   %s" % (controller.comRef.value, ))
comRef = np.array([com[i] + dcom[i] * dt + 0.5 * ddcomRef[i] * dt * dt for i in range(3)])
assert_almost_equal(controller.comRef.value, comRef)
