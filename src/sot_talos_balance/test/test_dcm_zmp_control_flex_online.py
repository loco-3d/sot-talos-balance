'''Test CoM admittance control as described in paper'''
from sys import argv
from time import sleep

from sot_talos_balance.utils.run_test_utils import (ask_for_confirmation, run_ft_calibration,
                                                    run_test, runCommandClient)

try:
    # Python 2
    input = raw_input  # noqa
except NameError:
    pass

walk_algo = "Naveau"

if (len(argv) == 1) or (len(argv) == 2 and argv[1] == "Naveau"):
    print("Starting script with Naveau walking algorithm.")
elif len(argv) == 2 and argv[1] == "Kajita":
    print("Starting script with with Kajita walking algorithm.")
    walk_algo = "Kajita"
else: 
    print("Usage: python test_dcm_zmp_control_flex_online.py {walking_algo:=[Naveau|Kajita]}")
    print("By default (no walking_algo given) the script starts using the Naveau algorithm")
    raise ValueError("Bad options")

runCommandClient('walk_algo = "' + walk_algo + '"')

flexi = ask_for_confirmation('Compensate flexibility?')
if flexi:
    print('Compensating flexibility')
    runCommandClient('flexi = False')
else:
    print('Not compensating flexibility')
    runCommandClient('flexi = False')

run_test('appli_dcm_zmp_control_flex_online.py')

run_ft_calibration('robot.ftc')

if flexi:
    fixed = ask_for_confirmation('Set fixed flexibility?')
    if fixed:
        runCommandClient('robot.hipComp.setFixedCompensation(0.020998)')
    input("Wait before activating flexibility")
    cmd_l = 'robot.hipComp.K_l.value = hipFlexCompConfig.flexibility_left'
    cmd_r = 'robot.hipComp.K_r.value = hipFlexCompConfig.flexibility_right'
    runCommandClient(cmd_l + '; ' + cmd_r)
else:
    runCommandClient('robot.hipComp.phase.value = 0')

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

if walk_algo == "Naveau":
    runCommandClient('robot.pg.parseCmd(":setVelReference  0.1 0.0 0.0")')
    runCommandClient('robot.pg.parseCmd(":SetAlgoForZmpTrajectory Naveau")')
    runCommandClient('robot.pg.parseCmd(":setDSFeetDistance 0.162")')
    runCommandClient('robot.pg.parseCmd(":NaveauOnline")')
    runCommandClient('robot.pg.parseCmd(":numberstepsbeforestop 2")')
    runCommandClient('robot.pg.parseCmd(":setfeetconstraint XY 0.091 0.0489")')
    runCommandClient('robot.pg.parseCmd(":deleteallobstacles")')
    runCommandClient('robot.pg.parseCmd(":feedBackControl false")')
    runCommandClient('robot.pg.velocitydes.value = (0.1, 0.0, 0.0)')
elif walk_algo == "Kajita":
    runCommandClient('robot.pg.parseCmd(":doublesupporttime 0.115")')
    runCommandClient('robot.pg.parseCmd(":singlesupporttime 0.9")')
    runCommandClient('robot.pg.parseCmd(":SetAlgoForZmpTrajectory Kajita")')
    runCommandClient('robot.pg.parseCmd(":StartOnLineStepSequencing 0.0 -0.09 0.0 0.0 0.1 0.18 0.0 0.0 0.1 -0.18 0.0 0.0 0.0 0.18 0.0 0.0")')
    runCommandClient('robot.pg.parseCmd(":StopOnLineStepSequencing")')

runCommandClient('robot.pg.parseCmd(":useDynamicFilter true")')

c = ask_for_confirmation('Execute trajectory?')
if c:
    print('Executing the trajectory')
    runCommandClient('robot.triggerPG.sin.value = 1')
    if walk_algo == "Naveau":
        input("Wait before stopping the trajectory")
        runCommandClient('robot.pg.velocitydes.value=(0.0,0.0,0.0)')
else:
    print('Not executing the trajectory')

input("Wait before dumping the data")

runCommandClient('dump_tracer(robot.tracer)')

print('Bye!')
