from __future__ import print_function

from dynamic_graph.sot_talos_balance.talos import control_manager_conf

from dynamic_graph.sot_talos_balance.talos import parameter_server_conf
import numpy as np
from dynamic_graph.sot_talos_balance.create_entities_utils import (
    ParameterServer,
    TalosControlManager,
)
from dynamic_graph.sot_talos_balance.utils.sot_utils import Bunch

dt = 0.001
conf = Bunch()
robot_name = "robot"
N_JOINTS = 38
u_max = 1.0

conf.param_server = parameter_server_conf
conf.control_manager = control_manager_conf

param_server = ParameterServer("param_server")
param_server.init(dt, conf.param_server.urdfFileName, robot_name)
param_server.setJointsUrdfToSot(np.array(conf.param_server.urdftosot))
param_server.setRightFootForceSensorXYZ(np.array(conf.param_server.rightFootSensorXYZ))
param_server.setRightFootSoleXYZ(np.array(conf.param_server.rightFootSoleXYZ))
for key in conf.param_server.mapJointNameToID:
    param_server.setNameToId(key, conf.param_server.mapJointNameToID[key])

cm = TalosControlManager("TalosControlManager")
cm.add_commands()
cm.init(dt, robot_name)
print("***Control manager initialized***")
cm.add_signals()
cm.u_max.value = np.array(N_JOINTS * (u_max,))
print(
    "u_max is set at {0} for all joints".format(
        u_max,
    )
)

# Control is Ok
print("*****************")
control_value = 0.5
u = np.array(N_JOINTS * [control_value])
cm.addCtrlMode("input")
cm.setCtrlMode("all", "input")
cm.add_signals()
cm.ctrl_input.value = u
print("Control input is set at {0} for all joints".format(control_value))
cm.u.recompute(1)
cm.u_safe.recompute(1)
assert (cm.u_safe.value == (control_value,) * N_JOINTS).all()
print("Safe control = control input")
print("*****************")
# Control is too big
control_value = 2.0
u = np.array(N_JOINTS * [control_value])
cm.ctrl_input.value = u
print("Control input is set at {0} for all joints".format(control_value))
cm.u.recompute(2)
cm.u_safe.recompute(2)
assert (cm.u_safe.value == (0.0,) * N_JOINTS).all()
print("Safe control = zero")
print("Control set to 0 forever")
print("*****************")

cm2 = TalosControlManager("TalosControlManager")
cm2.add_commands()
cm2.init(dt, robot_name)
print("***New Control manager initialized***")
cm2.add_signals()
cm.u_max.value = np.array(N_JOINTS * (u_max,))
print("u_max is set at {0} for all joints".format(u_max))

# Control is Ok
print("*****************")
control_value = 0.5
u = np.array(N_JOINTS * [control_value])
cm2.addCtrlMode("input")
cm2.setCtrlMode("all", "input")
cm2.ctrl_input.value = u
print("Control input is set at {0} for all joints".format(control_value))
cm2.u.recompute(3)
cm2.u_safe.recompute(3)
assert (cm2.u_safe.value == (control_value,) * N_JOINTS).all()
print("Safe control = control input")
print("*****************")

# Fake emergency signal
emergency = 0
print("EMERGENCY = 0")
cm2.addEmergencyStopSIN("test")
cm2.add_signals()
cm2.emergencyStop_test.value = emergency
cm2.u.recompute(4)
cm2.u_safe.recompute(4)
assert (cm2.u_safe.value == (control_value,) * N_JOINTS).all()
print("Safe control = control input")
emergency = 1
print("EMERGENCY = 1")
cm2.emergencyStop_test.value = emergency
cm2.u.recompute(5)
cm2.u_safe.recompute(5)
assert (cm2.u_safe.value == (0.0,) * N_JOINTS).all()
print("Safe control = zero")
print("Control set to 0 forever")
print("*****************")
