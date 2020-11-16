from __future__ import print_function

import dynamic_graph.sot_talos_balance.talos.parameter_server_conf as param_server_conf
import numpy as np
import pinocchio as pin
from dynamic_graph.sot_talos_balance.create_entities_utils import Bunch, DcmEstimator, ParameterServer
from numpy.testing import assert_almost_equal

dt = 0.001
conf = Bunch()
robot_name = 'robot'

urdfPath = param_server_conf.urdfFileName
urdfDir = param_server_conf.model_path

model = pin.buildModelFromUrdf(urdfPath, pin.JointModelFreeFlyer())
model.lowerPositionLimit = np.concatenate((np.array([-1.] * 7), model.lowerPositionLimit[7:]))
model.upperPositionLimit = np.concatenate((np.array([-1.] * 7), model.upperPositionLimit[7:]))
data = model.createData()
q = pin.randomConfiguration(model)
v = pin.utils.rand(model.nv)

pin.centerOfMass(model, data, q, v)
print("Expected:")
print("CoM position value: {0}".format(tuple(data.com[0].flat)))
print("CoM velocity value: {0}".format(tuple(data.vcom[0].flat)))

param_server = ParameterServer("param_server")
param_server.init(dt, param_server_conf.urdfFileName, robot_name)
param_server.setJointsUrdfToSot(np.array(param_server_conf.urdftosot))
param_server.setRightFootForceSensorXYZ(np.array(param_server_conf.rightFootSensorXYZ))
param_server.setRightFootSoleXYZ(np.array(param_server_conf.rightFootSoleXYZ))

dcm_estimator = DcmEstimator('dcm_estimator')
dcm_estimator.q.value = q
dcm_estimator.v.value = v
dcm_estimator.init(dt, robot_name)
dcm_estimator.c.recompute(1)
dcm_estimator.dc.recompute(1)
print("Computed:")
print("CoM position value: {0}".format(dcm_estimator.c.value))
assert_almost_equal(data.com[0], np.array(dcm_estimator.c.value), 3)
print("CoM velocity value: {0}".format(dcm_estimator.dc.value))
assert_almost_equal(data.vcom[0], np.array(dcm_estimator.dc.value), 3)
