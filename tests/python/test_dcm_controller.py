from __future__ import print_function

import numpy as np
from dynamic_graph.sot_talos_balance.dcm_controller import DcmController
from numpy.testing import assert_almost_equal as assertApprox

controller = DcmController("ciao")

print("\nSignals (at creation):")
controller.displaySignals()

Kp = np.array([10.0, 10.0, 0.0])
Ki = np.array([1.0, 1.0, 0.0])
omega = 1
mass = 1
com = np.array([0.0, 0.0, 1.0])
dcm = np.array(3 * [0.0])
dcmDes = np.array(3 * [0.0])
zmpDes = np.array(3 * [0.0])
decayFactor = 0.1

controller.Kp.value = Kp
controller.Ki.value = Ki
controller.omega.value = omega
controller.mass.value = mass
controller.com.value = com
controller.dcm.value = dcm
controller.dcmDes.value = dcmDes
controller.zmpDes.value = zmpDes
controller.decayFactor.value = decayFactor

print()
print("Kp:       %s" % (controller.Kp.value, ))
print("Ki:       %s" % (controller.Ki.value, ))
print("omega:    %s" % (controller.omega.value, ))
print("mass:     %s" % (controller.mass.value, ))
print("com:      %s" % (controller.com.value, ))
print("dcm:      %s" % (controller.dcm.value, ))
print("dcmDes:   %s" % (controller.dcmDes.value, ))
print("zmpDes:   %s" % (controller.dcmDes.value, ))
print("decayFactor: %s" % (controller.decayFactor.value, ))

print("\n--------------------")

dt = 1

controller.init(dt)

controller.wrenchRef.recompute(0)

zmpRef = np.array(3 * [0.0])
wrenchRef = np.array([0.0, 0.0, 9.81, 0.0, 0.0, 0.0])

print()
print("zmpRef:  %s" % (controller.zmpRef.value, ))
assertApprox(controller.zmpRef.value, zmpRef)
print("wrenchRef: %s" % (controller.wrenchRef.value, ))
assertApprox(controller.wrenchRef.value, wrenchRef)

print("\n--------------------")

dcmDes = np.array([1.0, 0.0, 0.0])
controller.dcmDes.value = dcmDes

print("dcmDes:   %s" % (controller.dcmDes.value, ))

controller.wrenchRef.recompute(1)

zmpRef = np.array([-11.0, 0.0, 0.0])
wrenchRef = np.array([11.0, 0.0, 9.81, 0.0, float(com[2] * 11), 0.0])

print()
print("zmpRef:  %s" % (controller.zmpRef.value, ))
assertApprox(controller.zmpRef.value, zmpRef)
print("wrenchRef: %s" % (controller.wrenchRef.value, ))
assertApprox(controller.wrenchRef.value, wrenchRef)

print("\n--------------------")

controller.dcmDes.time += 1

controller.zmpRef.recompute(2)
controller.wrenchRef.recompute(2)

zmpRef = np.array([-12.0, 0.0, 0.0])
wrenchRef = np.array([12.0, 0.0, 9.81, 0.0, float(com[2] * 12), 0.0])

print()
print("zmpRef:  %s" % (controller.zmpRef.value, ))
assertApprox(controller.zmpRef.value, zmpRef)
print("wrenchRef: %s" % (controller.wrenchRef.value, ))
assertApprox(controller.wrenchRef.value, wrenchRef)
