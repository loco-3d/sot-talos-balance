from time import sleep

from dynamic_graph.sot_talos_balance.utils.run_test_utils import (
    evalCommandClient,
    run_test,
    runCommandClient,
)

run_test("appli_admittance_single_joint.py")

sleep(1.0)
runCommandClient("robot.admittance_control.tauDes.value = [target]")
runCommandClient(
    "robot.admittance_control.setPosition([ robot.device.state.value[QJOINT] ])"
)
runCommandClient("robot.sot.push(robot.taskJoint.task.name)")

sleep(5.0)
des_tau = evalCommandClient("target")
tau = evalCommandClient("robot.device.ptorque.value[JOINT]")

print("Desired torque: %f" % des_tau)
print("Current torque: %f" % tau)
