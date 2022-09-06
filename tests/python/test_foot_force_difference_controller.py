from __future__ import print_function

import numpy as np
from dynamic_graph.sot_talos_balance.foot_force_difference_controller import (
    FootForceDifferenceController,
)
from numpy.testing import assert_almost_equal

controller = FootForceDifferenceController("footController")
controller.init()

controller.dfzAdmittance.value = 1.0
controller.vdcFrequency.value = 0.0
controller.vdcDamping.value = 0.0

gainSwing = 1.0
gainStance = 2.0
gainDouble = 3.0

controller.wrenchRight.value = np.array([0.0] * 2 + [500.0] + [0.0] * 3)
controller.wrenchLeft.value = np.array([0.0] * 2 + [300.0] + [0.0] * 3)
controller.wrenchRightDes.value = np.array([0.0] * 2 + [400.0] + [0.0] * 3)
controller.wrenchLeftDes.value = np.array([0.0] * 2 + [400.0] + [0.0] * 3)

controller.gainSwing.value = gainSwing
controller.gainStance.value = gainStance
controller.gainDouble.value = gainDouble

print("---- Input ----")
print("wrenchRight:    %s" % str(controller.wrenchRight.value))
print("wrenchLeft:     %s" % str(controller.wrenchLeft.value))
print("wrenchRightDes: %s" % str(controller.wrenchRightDes.value))
print("wrenchLeftDes:  %s" % str(controller.wrenchLeftDes.value))
print()

print("gainSwing:  %s" % str(controller.gainSwing.value))
print("gainStance: %s" % str(controller.gainStance.value))
print("gainDouble: %s" % str(controller.gainDouble.value))
print()

print("---- Double support ----")

controller.phase.value = 0

controller.posRightDes.value = np.eye(4)
controller.posLeftDes.value = np.eye(4)
controller.posRight.value = np.eye(4)
controller.posLeft.value = np.eye(4)

controller.vRight.recompute(0)
controller.vLeft.recompute(0)
controller.gainRight.recompute(0)
controller.gainLeft.recompute(0)

# There is more pressure on the right foot.
# Therefore, the right foot must go up to reduce it
vRight = [0.0] * 2 + [100.0] + [0.0] * 3
vLeft = [0.0] * 2 + [-100.0] + [0.0] * 3

print("Expected vRight: %s" % str(vRight))
print("Actual vRight:   %s" % str(controller.vRight.value))
print("Expected vLeft:  %s" % str(vLeft))
print("Actual vLeft:    %s" % str(controller.vLeft.value))
print()

assert_almost_equal(vRight, controller.vRight.value)
assert_almost_equal(vLeft, controller.vLeft.value)

print("gainRight:   %s" % str(controller.gainRight.value))
print("gainLeft:    %s" % str(controller.gainLeft.value))
print()

assert_almost_equal(gainDouble, controller.gainRight.value)
assert_almost_equal(gainDouble, controller.gainLeft.value)

print("---- Left support ----")
controller.phase.value = 1

controller.vRight.recompute(1)
controller.vLeft.recompute(1)
controller.gainRight.recompute(1)
controller.gainLeft.recompute(1)
print("vRight:   %s" % str(controller.vRight.value))
print("vLeft:    %s" % str(controller.vLeft.value))
print()

assert_almost_equal([0.0] * 6, controller.vRight.value)
assert_almost_equal([0.0] * 6, controller.vLeft.value)

print("gainRight:   %s" % str(controller.gainRight.value))
print("gainLeft:    %s" % str(controller.gainLeft.value))
print()

assert_almost_equal(gainSwing, controller.gainRight.value)
assert_almost_equal(gainStance, controller.gainLeft.value)

print("---- Right support ----")
controller.phase.value = -1

controller.vRight.recompute(2)
controller.vLeft.recompute(2)
controller.gainRight.recompute(2)
controller.gainLeft.recompute(2)
print("vRight:   %s" % str(controller.vRight.value))
print("vLeft:    %s" % str(controller.vLeft.value))
print()

assert_almost_equal([0.0] * 6, controller.vRight.value)
assert_almost_equal([0.0] * 6, controller.vLeft.value)

print("gainRight:   %s" % str(controller.gainRight.value))
print("gainLeft:    %s" % str(controller.gainLeft.value))
print()

assert_almost_equal(gainStance, controller.gainRight.value)
assert_almost_equal(gainSwing, controller.gainLeft.value)
