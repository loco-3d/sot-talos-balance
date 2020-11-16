from math import sqrt

import dynamic_graph.sot_talos_balance.talos.parameter_server_conf as param_server_conf
import numpy as np
import pinocchio as pin
from dynamic_graph.sot_talos_balance.create_entities_utils import DcmController, create_parameter_server
from numpy.testing import assert_almost_equal

# --- General ---
print("--- General ---")

dt = 0.001
robot_name = 'robot'

halfSitting = np.array([
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
    0.,
    0.  # Head
])

q = halfSitting
print("q:", q)

urdfPath = param_server_conf.urdfFileName
urdfDir = param_server_conf.model_path

model = pin.buildModelFromUrdf(urdfPath, pin.JointModelFreeFlyer())
data = model.createData()
com = pin.centerOfMass(model, data, q)
pin.updateFramePlacements(model, data)
m = data.mass[0]
h = float(com[2])
g = 9.81
omega = sqrt(g / h)

leftName = param_server_conf.footFrameNames['Left']
leftId = model.getFrameId(leftName)
leftPos = data.oMf[leftId]

rightName = param_server_conf.footFrameNames['Right']
rightId = model.getFrameId(rightName)
rightPos = data.oMf[rightId]

centerTranslation = (data.oMf[rightId].translation + data.oMf[leftId].translation) / 2 + np.array(
    param_server_conf.rightFootSoleXYZ)
centerPos = pin.SE3(rightPos.rotation, centerTranslation)
comRel = centerPos.actInv(com)

fz = m * g
force = [0.0, 0.0, fz]
tau = np.cross(comRel, np.array(force), axis=0)
wrench = force + tau.flatten().tolist()

print("desired wrench: %s" % str(wrench))

# --- Desired CoM, DCM and ZMP
comDes = tuple(comRel.flatten().tolist())
dcmDes = np.array(comDes)
zmpDes = np.array(comDes[:2] + (0.0, ))

# --- Parameter server ---
print("--- Parameter server ---")

param_server = create_parameter_server(param_server_conf, dt)

# --- DCM controller
Kp_dcm = np.array([0.0, 0.0, 0.0])
Ki_dcm = np.array([0.0, 0.0, 0.0])
gamma_dcm = 0.2

dcm_controller = DcmController("dcmCtrl")

dcm_controller.Kp.value = Kp_dcm
dcm_controller.Ki.value = Ki_dcm
dcm_controller.decayFactor.value = gamma_dcm
dcm_controller.mass.value = m
dcm_controller.omega.value = omega

dcm_controller.com.value = np.array(comDes)
dcm_controller.dcm.value = np.array(comDes)

dcm_controller.zmpDes.value = zmpDes
dcm_controller.dcmDes.value = dcmDes

dcm_controller.init(dt)

dcm_controller.wrenchRef.recompute(0)

print("reference wrench: %s" % str(dcm_controller.wrenchRef.value))
assert_almost_equal(wrench, dcm_controller.wrenchRef.value, 3)
