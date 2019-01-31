from sot_talos_balance.create_entities_utils import *
from dynamic_graph import plug
import dynamic_graph as dg
from dynamic_graph.sot.core import SOT
import numpy as np
from time import sleep
import os
from IPython import embed
from sot_talos_balance.parameter_server             import ParameterServer
from sot_talos_balance.utils.sot_utils              import Bunch
from sot_talos_balance.dcm_estimator                import DcmEstimator
from sot_talos_balance.utils.sot_utils              import Bunch
import sot_talos_balance.talos.control_manager_conf as control_manager_conf


dt = 0.001
conf = Bunch()
robot_name = 'robot'

conf.param_server = control_manager_conf
param_server = ParameterServer("param_server")     
param_server.init(dt, conf.param_server.urdfFileName, robot_name)
param_server.setRightFootForceSensorXYZ(conf.param_server.rightFootSensorXYZ)
param_server.setRightFootSoleXYZ(conf.param_server.rightFootSoleXYZ)

dcm_estimator = DcmEstimator('dcm_estimator')
dcm_estimator.q.value = np.random.randn(38)
dcm_estimator.v.value = np.random.randn(38)
dcm_estimator.init(dt, robot_name)
dcm_estimator.c.recompute(1)
dcm_estimator.dc.recompute(1)
print "CoM position value: {0}".format(dcm_estimator.c.value)
print "CoM velocity value: {0}".format(dcm_estimator.dc.value)

embed()