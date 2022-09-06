import dynamic_graph.sot_talos_balance.talos.base_estimator_conf as base_estimator_conf
import dynamic_graph.sot_talos_balance.talos.parameter_server_conf as param_server_conf
import numpy as np
import pinocchio as pin
from dynamic_graph.sot_talos_balance.create_entities_utils import (
    DcmEstimator,
    TalosBaseEstimator,
    create_parameter_server,
    plug,
)
from dynamic_graph.sot_talos_balance.euler_to_quat import EulerToQuat
from dynamic_graph.sot_talos_balance.simple_reference_frame import SimpleReferenceFrame
from dynamic_graph.sot_talos_balance.state_transformation import StateTransformation
from numpy.testing import assert_almost_equal

# --- General ---
print("--- General ---")

dt = 0.001
robot_name = "robot"

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
com = pin.centerOfMass(model, data, q)
pin.updateFramePlacements(model, data)
m = data.mass[0]

print("com:")
print(com.flatten().tolist()[0])

leftName = param_server_conf.footFrameNames["Left"]
leftId = model.getFrameId(leftName)
leftPos = data.oMf[leftId]
print("%s: %d" % (leftName, leftId))
print(leftPos)

rightName = param_server_conf.footFrameNames["Right"]
rightId = model.getFrameId(rightName)
rightPos = data.oMf[rightId]
print("%s: %d" % (rightName, rightId))
print(rightPos)

g = 9.81
fz = m * g / 2.0
forceLeft = np.array([0.0, 0.0, fz])
forceRight = np.array([0.0, 0.0, fz])
lever = float(com[0] - rightPos.translation[0])
tauy = -fz * lever
wrenchLeft = np.array(forceLeft.tolist() + [0.0, tauy, 0.0])
wrenchRight = np.array(forceRight.tolist() + [0.0, tauy, 0.0])

# --- Parameter server ---
print("--- Parameter server ---")

param_server = create_parameter_server(param_server_conf, dt)

# --- Base estimator ---
print("--- Base estimator ---")

conf = base_estimator_conf
base_estimator = TalosBaseEstimator("base_estimator")
base_estimator.init(dt, robot_name)

base_estimator.joint_positions.value = halfSitting[7:]
base_estimator.forceLLEG.value = wrenchLeft
base_estimator.forceRLEG.value = wrenchRight
base_estimator.dforceLLEG.value = np.array([0.0] * 6)
base_estimator.dforceRLEG.value = np.array([0.0] * 6)
base_estimator.joint_velocities.value = np.array([0.0] * (model.nv - 6))
base_estimator.imu_quaternion.value = np.array([0.0] * 3 + [1.0])
base_estimator.gyroscope.value = np.array([0.0] * 3)
base_estimator.accelerometer.value = np.array([0.0] * 3)

base_estimator.K_fb_feet_poses.value = conf.K_fb_feet_poses
base_estimator.w_lf_in.value = conf.w_lf_in
base_estimator.w_rf_in.value = conf.w_rf_in
# base_estimator.set_imu_weight(conf.w_imu) # TEMP!
base_estimator.set_imu_weight(0.0)
base_estimator.set_stiffness_right_foot(np.array(conf.K))
base_estimator.set_stiffness_left_foot(np.array(conf.K))
base_estimator.set_zmp_std_dev_right_foot(conf.std_dev_zmp)
base_estimator.set_zmp_std_dev_left_foot(conf.std_dev_zmp)
base_estimator.set_normal_force_std_dev_right_foot(conf.std_dev_fz)
base_estimator.set_normal_force_std_dev_left_foot(conf.std_dev_fz)
base_estimator.set_zmp_margin_right_foot(conf.zmp_margin)
base_estimator.set_zmp_margin_left_foot(conf.zmp_margin)
base_estimator.set_normal_force_margin_right_foot(conf.normal_force_margin)
base_estimator.set_normal_force_margin_left_foot(conf.normal_force_margin)
base_estimator.set_right_foot_sizes(np.array(conf.RIGHT_FOOT_SIZES))
base_estimator.set_left_foot_sizes(np.array(conf.LEFT_FOOT_SIZES))

base_estimator.q.recompute(0)
print(base_estimator.q.value)
print(len(base_estimator.q.value))

# --- Reference frame
rf = SimpleReferenceFrame("rf")
rf.init(robot_name)
rf.footLeft.value = leftPos.homogeneous
rf.footRight.value = rightPos.homogeneous

# --- State transformation
stf = StateTransformation("stf")
stf.init()
plug(rf.referenceFrame, stf.referenceFrame)
plug(base_estimator.q, stf.q_in)
plug(base_estimator.v, stf.v_in)

stf.q.recompute(0)
print(stf.q.value)
print(len(stf.q.value))

stf.v.recompute(0)
print(stf.v.value)
print(len(stf.v.value))

# --- Conversion ---
print("--- Conversion ---")

e2q = EulerToQuat("e2q")
plug(stf.q, e2q.euler)
e2q.quaternion.recompute(0)
print(e2q.quaternion.value)
print(len(e2q.quaternion.value))
q_est = np.array(e2q.quaternion.value)

# temp
if q_est[6] < 0 and q[6] > 0:
    print("Warning: quaternions have different signs")
    q_est[3:7] = -q_est[3:7]

print(q_est.T)
assert_almost_equal(q, q_est, 3)

# --- Raw q difference ---
print("--- Raw q difference ---")
q_diff = q_est - q
print(q_diff.flatten().tolist()[0])

# --- DCM estimator ---
print("--- DCM estimator ---")

dcm_estimator = DcmEstimator("dcm_estimator")
dcm_estimator.init(dt, robot_name)
plug(e2q.quaternion, dcm_estimator.q)
plug(base_estimator.v, dcm_estimator.v)
dcm_estimator.c.recompute(0)
print(dcm_estimator.c.value)

# --- Direct CoM ---
print("--- Direct CoM ---")
print(com.flatten().tolist()[0])

# --- CoM difference ---
print("--- CoM difference ---")
com_rawdiff = np.matrix(dcm_estimator.c.value).T - com
print(com_rawdiff.flatten().tolist()[0])

assert_almost_equal(dcm_estimator.c.value, com, 3)
