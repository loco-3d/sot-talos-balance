from __future__ import print_function

import numpy as np
from dynamic_graph.sot_talos_balance.ankle_joint_selector import AnkleJointSelector

# TODO: print value ? since cpp11 branch, the getter doesn't seem to work

ajs = AnkleJointSelector('ajs')
ajs.init(38)
ajs.rightRollDecoupled.value = np.array([1.])
ajs.rightRollCoupled.value = np.array([2.])
ajs.rightPitchDecoupled.value = np.array([3.])
ajs.rightPitchCoupled.value = np.array([4.])
ajs.leftRollDecoupled.value = np.array([5.])
ajs.leftRollCoupled.value = np.array([6.])
ajs.leftPitchDecoupled.value = np.array([7.])
ajs.leftPitchCoupled.value = np.array([8.])

time = 0
print("\nRight support")
ajs.phase.value = -1
ajs.phase.time = time
print("Phase: %d" % ajs.phase.value)
ajs.selecRight.recompute(time)
# print("Right: %s" % ajs.selecRight.value, end='')
ajs.rightRoll.recompute(time)
print("Right roll:   %s" % ajs.rightRoll.value)
ajs.rightPitch.recompute(time)
print("Right pitch:  %s" % ajs.rightPitch.value)
ajs.selecLeft.recompute(time)
# print("Left:  %s" % ajs.selecLeft.value, end='')
ajs.leftRoll.recompute(time)
print("Left roll:   %s" % ajs.leftRoll.value)
ajs.leftPitch.recompute(time)
print("Left pitch:  %s" % ajs.leftPitch.value)

time += 1
print("\nDouble support")
ajs.phase.value = 0
ajs.phase.time = time
print("Phase: %d" % ajs.phase.value)
ajs.selecRight.recompute(time)
# print("Right: %s" % ajs.selecRight.value, end='')
ajs.rightRoll.recompute(time)
print("Right roll:   %s" % ajs.rightRoll.value)
ajs.rightPitch.recompute(time)
print("Right pitch:  %s" % ajs.rightPitch.value)
ajs.selecLeft.recompute(time)
# print("Left:  %s" % ajs.selecLeft.value, end='')
ajs.leftRoll.recompute(time)
print("Left roll:   %s" % ajs.leftRoll.value)
ajs.leftPitch.recompute(time)
print("Left pitch:  %s" % ajs.leftPitch.value)

time += 1
print("\nLeft support")
ajs.phase.value = 1
ajs.phase.time = time
print("Phase: %d" % ajs.phase.value)
ajs.selecRight.recompute(time)
# print("Right: %s" % ajs.selecRight.value, end='')
ajs.rightRoll.recompute(time)
print("Right roll:   %s" % ajs.rightRoll.value)
ajs.rightPitch.recompute(time)
print("Right pitch:  %s" % ajs.rightPitch.value)
ajs.selecLeft.recompute(time)
# print("Left:  %s" % ajs.selecLeft.value, end='')
ajs.leftRoll.recompute(time)
print("Left roll:   %s" % ajs.leftRoll.value)
ajs.leftPitch.recompute(time)
print("Left pitch:  %s" % ajs.leftPitch.value)
