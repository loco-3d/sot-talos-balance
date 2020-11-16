import dynamic_graph.sot_talos_balance.talos.ft_calibration_conf as conf
import numpy as np
from dynamic_graph.sot_talos_balance.ft_calibration import FtCalibration
from numpy.testing import assert_almost_equal

robot_name = 'robot'
ftc = FtCalibration('ftc')
ftc.init(robot_name)
rfw = conf.rfw
lfw = conf.lfw
ftc.setLeftFootWeight(lfw)
ftc.setRightFootWeight(rfw)
print('Weights of both feet set to {0}'.format(rfw))

print('The robot should be in the air.')

ftc.right_foot_force_in.value = np.array([1, 1, 8, 1, 1, 1])
ftc.left_foot_force_in.value = np.array([1, 1, 6, 1, 1, 1])

print("Let's calibrate the ft sensors...")
ftc.calibrateFeetSensor()
for i in range(2, 1003):
    ftc.right_foot_force_in.value = np.random.randn(6) * 0.0000001 + [1, 1, 8, 1, 1, 1]
    ftc.left_foot_force_in.value = np.random.randn(6) * 0.0000001 + [1, 1, 6, 1, 1, 1]
    ftc.right_foot_force_out.recompute(i)
    ftc.left_foot_force_out.recompute(i)

assert_almost_equal(ftc.right_foot_force_out.value, np.array((0, 0, -rfw, 0, 0, 0)), 5)
assert_almost_equal(ftc.left_foot_force_out.value, np.array((0, 0, -lfw, 0, 0, 0)), 5)

print("Ft sensors calibrated!")
print("Value outputed after calibration:")
print(ftc.right_foot_force_out.value)
print(ftc.left_foot_force_out.value)
