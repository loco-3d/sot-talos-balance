'''Test CoM admittance control as described in paper'''
from sys import argv
from time import sleep

from dynamic_graph.sot_talos_balance.utils.run_test_utils import (ask_for_confirmation, get_file_folder, run_ft_calibration,
                                                                  run_test, runCommandClient)

try:
    # Python 2
    input = raw_input  # noqa
except NameError:
    pass

test_folder, sot_talos_balance_folder = get_file_folder(argv)

run_test('appli_dcm_zmp_control_mlp.py')

run_ft_calibration('robot.ftc')

input("Wait before running the test")

# Connect ZMP reference and reset controllers
print('Connect ZMP reference')
runCommandClient('plug(robot.zmp_estimator.emergencyStop,robot.cm.emergencyStop_zmp)')
runCommandClient('plug(robot.dcm_control.zmpRef,robot.com_admittance_control.zmpDes)')
runCommandClient('robot.com_admittance_control.setState(robot.wp.comDes.value,[0.0,0.0,0.0])')
runCommandClient('robot.com_admittance_control.Kp.value = Kp_adm')
runCommandClient('robot.dcm_control.resetDcmIntegralError()')
runCommandClient('robot.dcm_control.Ki.value = Ki_dcm')
runCommandClient('robot.dcm_control.Kz.value = Kz_dcm')

if test_folder is not None:
    c = ask_for_confirmation('Execute trajectory?')
    if c:
        print('Executing the trajectory')
        runCommandClient('set_trigger(robot, True)')
    else:
        print('Not executing the trajectory')
input("Wait before dumping the data")

runCommandClient('dump_tracer(robot.tracer)')

print('Bye!')
