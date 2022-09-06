from math import sqrt

import dynamic_graph.sot_talos_balance.talos.parameter_server_conf as param_server_conf
import numpy as np
import pinocchio as pin
from dynamic_graph.sot_talos_balance.create_entities_utils import (
    create_parameter_server,
    plug,
)
from dynamic_graph.sot_talos_balance.dummy_walking_pattern_generator import (
    DummyWalkingPatternGenerator,
)
from dynamic_graph.sot_talos_balance.simple_reference_frame import SimpleReferenceFrame
from numpy.testing import assert_almost_equal

# --- General ---
print("--- General ---")

dt = 0.001

halfSitting = np.array(
    [
        0.0,
        0.0,
        1.018213,
        0.00,
        0.0,
        0.0,
        1.0,  # Free flyer
        0.0,
        0.0,
        -0.411354,
        0.859395,
        -0.448041,
        -0.001708,  # Left Leg
        0.0,
        0.0,
        -0.411354,
        0.859395,
        -0.448041,
        -0.001708,  # Right Leg
        0.0,
        0.006761,  # Chest
        0.25847,
        0.173046,
        -0.0002,
        -0.525366,
        0.0,
        -0.0,
        0.1,
        -0.005,  # Left Arm
        -0.25847,
        -0.173046,
        0.0002,
        -0.525366,
        0.0,
        0.0,
        0.1,
        -0.005,  # Right Arm
        0.0,
        0.0,  # Head
    ]
)

q = halfSitting
print("q:", q)

urdfPath = param_server_conf.urdfFileName
urdfDir = param_server_conf.model_path

model = pin.buildModelFromUrdf(urdfPath, pin.JointModelFreeFlyer())
data = model.createData()

# --- Desired CoM
com = pin.centerOfMass(model, data, q)
pin.updateFramePlacements(model, data)
vcom = np.array((0.0, 0.0, 0.0))
acom = np.array((0.0, 0.0, 0.0))

comDes = com

print("Absolute desired com:")
print(com)

# --- Translation
# comDes = list(comDes)
# comDes[0] += 0.01
# comDes[1] += 0.01
# comDes = tuple(comDes)

# --- Conversion to center of feet reference frame

leftName = param_server_conf.footFrameNames["Left"]
leftId = model.getFrameId(leftName)
leftPos = data.oMf[leftId]
rightName = param_server_conf.footFrameNames["Right"]
rightId = model.getFrameId(rightName)
rightPos = data.oMf[rightId]
centerTranslation = (
    data.oMf[rightId].translation + data.oMf[leftId].translation
) / 2 + np.array(param_server_conf.rightFootSoleXYZ).T
centerPos = pin.SE3(rightPos.rotation, centerTranslation)

comDes = centerPos.actInv(np.array(comDes))

# --- Desired DCM and ZMP
dcmDes = comDes
zmpDes = np.array(comDes[:2].tolist() + [0.0])

# --- Desired CoM velocity
vcomDes = (0.0, 0.0, 0.0)

# --- Desired CoM acceleration
acomDes = (0.0, 0.0, 0.0)

# --- Print desired values
print("Relative CoM:")
print(comDes)
print("Relative CoM velocity:")
print(vcomDes)
print("Relative CoM acceleration:")
print(acomDes)
print("Relative DCM:")
print(dcmDes)
print("Relative ZMP:")
print(zmpDes)

# --- Pendulum parameters
robot_name = "robot"
h = com[2]
g = 9.81
omega = sqrt(g / h)

# --- Parameter server ---
print("--- Parameter server ---")

param_server = create_parameter_server(param_server_conf, dt)

# --- Reference frame ---
print("--- Reference frame ---")

rf = SimpleReferenceFrame("rf")
rf.init(robot_name)
rf.footLeft.value = leftPos.homogeneous
rf.footRight.value = rightPos.homogeneous
rf.reset.value = 1

# --- Dummy Walking Pattern Generator ---
print("--- Dummy Walking Pattern Generator ---")

wp = DummyWalkingPatternGenerator("dummy_wp")
wp.init()
plug(rf.referenceFrame, wp.referenceFrame)
wp.omega.value = omega
wp.footLeft.value = leftPos.homogeneous
wp.footRight.value = rightPos.homogeneous
wp.com.value = com
wp.vcom.value = vcom
wp.acom.value = acom

wp.comDes.recompute(0)
wp.dcmDes.recompute(0)
wp.zmpDes.recompute(0)

# --- Output
print("Desired CoM:")
print(wp.comDes.value)
assert_almost_equal(comDes, wp.comDes.value, 3)
print("Desired CoM velocity:")
print(wp.vcomDes.value)
assert_almost_equal(vcomDes, wp.vcomDes.value, 3)
print("Desired CoM acceleration:")
print(wp.acomDes.value)
assert_almost_equal(acomDes, wp.acomDes.value, 3)
print("Desired DCM:")
print(wp.dcmDes.value)
assert_almost_equal(dcmDes, wp.dcmDes.value, 3)
print("Desired ZMP:")
print(wp.zmpDes.value)
assert_almost_equal(zmpDes, wp.zmpDes.value, 3)
