from sot_talos_balance.create_entities_utils import *
import sot_talos_balance.talos.parameter_server_conf   as param_server_conf
import sot_talos_balance.talos.control_manager_conf    as cm_conf
import sot_talos_balance.talos.base_estimator_conf     as base_estimator_conf
import sot_talos_balance.talos.ft_calibration_conf     as ft_conf
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph.sot.core import Task, FeaturePosture
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT
from math import sqrt
import numpy as np

from dynamic_graph.tracer_real_time import TracerRealTime

from dynamic_graph.sot.dynamics_pinocchio import DynamicPinocchio

robot.timeStep = robot.device.getTimeStep()
dt = robot.timeStep

# --- Pendulum parameters
robot_name='robot'
robot.dynamic.com.recompute(0)
robotDim = robot.dynamic.getDimension()
mass = robot.dynamic.data.mass[0]
h = robot.dynamic.com.value[2]
g = 9.81
omega = sqrt(g/h)

# --- Parameter server
robot.param_server = create_parameter_server(param_server_conf,dt)

# --- Initial feet and waist
robot.dynamic.createOpPoint('LF',robot.OperationalPointsMap['left-ankle'])
robot.dynamic.createOpPoint('RF',robot.OperationalPointsMap['right-ankle'])
robot.dynamic.createOpPoint('WT',robot.OperationalPointsMap['waist'])
robot.dynamic.LF.recompute(0)
robot.dynamic.RF.recompute(0)
robot.dynamic.WT.recompute(0)

# -------------------------- DESIRED TRAJECTORY --------------------------

from rospkg import RosPack
rospack = RosPack()

data_folder = rospack.get_path('sot_talos_balance')+"/data/"
folder = data_folder + test_folder + '/'

# --- Trajectory generators

# --- General trigger
robot.triggerTrajGen = BooleanIdentity('triggerTrajGen')
robot.triggerTrajGen.sin.value = 0

# --- CoM
robot.comTrajGen = create_com_trajectory_generator(dt, robot)
robot.comTrajGen.x.recompute(0) # trigger computation of initial value
robot.comTrajGen.playTrajectoryFile(folder+'CoM.dat')
plug(robot.triggerTrajGen.sout, robot.comTrajGen.trigger)

# --- Left foot
robot.lfTrajGen  = create_pose_rpy_trajectory_generator(dt, robot, 'LF')
robot.lfTrajGen.x.recompute(0) # trigger computation of initial value

robot.lfToMatrix = PoseRollPitchYawToMatrixHomo('lf2m')
plug(robot.lfTrajGen.x, robot.lfToMatrix.sin)
robot.lfTrajGen.playTrajectoryFile(folder+'LeftFoot.dat')
plug(robot.triggerTrajGen.sout, robot.lfTrajGen.trigger)

# --- Right foot
robot.rfTrajGen  = create_pose_rpy_trajectory_generator(dt, robot, 'RF')
robot.rfTrajGen.x.recompute(0) # trigger computation of initial value

robot.rfToMatrix = PoseRollPitchYawToMatrixHomo('rf2m')
plug(robot.rfTrajGen.x, robot.rfToMatrix.sin)
robot.rfTrajGen.playTrajectoryFile(folder+'RightFoot.dat')
plug(robot.triggerTrajGen.sout, robot.rfTrajGen.trigger)

# --- ZMP
robot.zmpTrajGen = create_zmp_trajectory_generator(dt,robot)
robot.zmpTrajGen.x.recompute(0) # trigger computation of initial value
robot.zmpTrajGen.playTrajectoryFile(folder+'ZMP.dat')
plug(robot.triggerTrajGen.sout, robot.zmpTrajGen.trigger)

# --- Waist
robot.waistTrajGen = create_orientation_rpy_trajectory_generator(dt, robot, 'WT')
robot.waistTrajGen.x.recompute(0) # trigger computation of initial value

robot.waistMix = Mix_of_vector("waistMix")
robot.waistMix.setSignalNumber(3)
robot.waistMix.addSelec(1, 0, 3)
robot.waistMix.addSelec(2, 3, 3)
robot.waistMix.default.value = [0.0]*6
robot.waistMix.signal("sin1").value = [0.0]*3
plug(robot.waistTrajGen.x, robot.waistMix.signal("sin2"))

robot.waistToMatrix = PoseRollPitchYawToMatrixHomo('w2m')
plug(robot.waistMix.sout, robot.waistToMatrix.sin)
robot.waistTrajGen.playTrajectoryFile(folder+'WaistOrientation.dat')
plug(robot.triggerTrajGen.sout, robot.waistTrajGen.trigger)

# --- Interface with controller entities

wp = DummyWalkingPatternGenerator('dummy_wp')
wp.init()
wp.omega.value = omega
#wp.waist.value = robot.dynamic.WT.value          # wait receives a full homogeneous matrix, but only the rotational part is controlled
#wp.footLeft.value = robot.dynamic.LF.value
#wp.footRight.value = robot.dynamic.RF.value
#wp.com.value  = robot.dynamic.com.value
#wp.vcom.value = [0.]*3
#wp.acom.value = [0.]*3
plug(robot.waistToMatrix.sout, wp.waist)
plug(robot.lfToMatrix.sout, wp.footLeft)
plug(robot.rfToMatrix.sout, wp.footRight)
plug(robot.comTrajGen.x, wp.com)
plug(robot.comTrajGen.dx, wp.vcom)
plug(robot.comTrajGen.ddx, wp.acom)
#plug(robot.zmpTrajGen.x, wp.zmp)

robot.wp = wp

# --- Compute the values to use them in initialization
robot.wp.comDes.recompute(0)
robot.wp.dcmDes.recompute(0)
robot.wp.zmpDes.recompute(0)

# -------------------------- ESTIMATION --------------------------

# --- Base Estimation
robot.device_filters          = create_device_filters(robot, dt)
robot.imu_filters             = create_imu_filters(robot, dt)
robot.base_estimator          = create_base_estimator(robot, dt, base_estimator_conf)

from dynamic_graph.sot.core import MatrixHomoToPoseQuaternion
robot.m2qLF = MatrixHomoToPoseQuaternion('m2qLF')
plug(robot.dynamic.LF, robot.m2qLF.sin)
plug(robot.m2qLF.sout, robot.base_estimator.lf_ref_xyzquat)
robot.m2qRF = MatrixHomoToPoseQuaternion('m2qRF')
plug(robot.dynamic.RF, robot.m2qRF.sin)
plug(robot.m2qRF.sout, robot.base_estimator.rf_ref_xyzquat)

# robot.be_filters              = create_be_filters(robot, dt)

# -------------------------- SOT CONTROL --------------------------

# --- Upper body
robot.taskUpperBody = Task ('task_upper_body')
robot.taskUpperBody.feature = FeaturePosture('feature_upper_body')

q = list(robot.dynamic.position.value)
robot.taskUpperBody.feature.state.value = q
robot.taskUpperBody.feature.posture.value = q

# robotDim = robot.dynamic.getDimension() # 38
robot.taskUpperBody.feature.selectDof(18,True)
robot.taskUpperBody.feature.selectDof(19,True)
robot.taskUpperBody.feature.selectDof(20,True)
robot.taskUpperBody.feature.selectDof(21,True)
robot.taskUpperBody.feature.selectDof(22,True)
robot.taskUpperBody.feature.selectDof(23,True)
robot.taskUpperBody.feature.selectDof(24,True)
robot.taskUpperBody.feature.selectDof(25,True)
robot.taskUpperBody.feature.selectDof(26,True)
robot.taskUpperBody.feature.selectDof(27,True)
robot.taskUpperBody.feature.selectDof(28,True)
robot.taskUpperBody.feature.selectDof(29,True)
robot.taskUpperBody.feature.selectDof(30,True)
robot.taskUpperBody.feature.selectDof(31,True)
robot.taskUpperBody.feature.selectDof(32,True)
robot.taskUpperBody.feature.selectDof(33,True)
robot.taskUpperBody.feature.selectDof(34,True)
robot.taskUpperBody.feature.selectDof(35,True)
robot.taskUpperBody.feature.selectDof(36,True)
robot.taskUpperBody.feature.selectDof(37,True)

robot.taskUpperBody.controlGain.value = 100.0
robot.taskUpperBody.add(robot.taskUpperBody.feature.name)
plug(robot.dynamic.position, robot.taskUpperBody.feature.state)

# --- CONTACTS
#define contactLF and contactRF
robot.contactLF = MetaTaskKine6d('contactLF',robot.dynamic,'LF',robot.OperationalPointsMap['left-ankle'])
robot.contactLF.feature.frame('desired')
robot.contactLF.gain.setConstant(300)
plug(robot.wp.footLeftDes, robot.contactLF.featureDes.position) #.errorIN?
locals()['contactLF'] = robot.contactLF

robot.contactRF = MetaTaskKine6d('contactRF',robot.dynamic,'RF',robot.OperationalPointsMap['right-ankle'])
robot.contactRF.feature.frame('desired')
robot.contactRF.gain.setConstant(300)
plug(robot.wp.footRightDes, robot.contactRF.featureDes.position) #.errorIN?
locals()['contactRF'] = robot.contactRF

# --- COM height
robot.taskCom = MetaTaskKineCom(robot.dynamic)
plug(robot.wp.comDes,robot.taskCom.featureDes.errorIN)
robot.taskCom.task.controlGain.value = 100.

# --- Waist
robot.keepWaist = MetaTaskKine6d('keepWaist',robot.dynamic,'WT',robot.OperationalPointsMap['waist'])
robot.keepWaist.feature.frame('desired')
robot.keepWaist.gain.setConstant(300)
plug(robot.wp.waistDes, robot.keepWaist.featureDes.position)
robot.keepWaist.feature.selec.value = '111000'
locals()['keepWaist'] = robot.keepWaist

# --- SOT solver
robot.sot = SOT('sot')
robot.sot.setSize(robot.dynamic.getDimension())

# --- Plug SOT control to device
plug(robot.sot.control,robot.device.control)

robot.sot.push(robot.taskUpperBody.name)
robot.sot.push(robot.contactRF.task.name)
robot.sot.push(robot.contactLF.task.name)
robot.sot.push(robot.taskCom.task.name)
robot.sot.push(robot.keepWaist.task.name)

# --- Fix robot.dynamic inputs
plug(robot.device.velocity,robot.dynamic.velocity)
from dynamic_graph.sot.core import Derivator_of_Vector
robot.dvdt = Derivator_of_Vector("dv_dt")
robot.dvdt.dt.value = dt
plug(robot.device.velocity,robot.dvdt.sin)
plug(robot.dvdt.sout,robot.dynamic.acceleration)

# -------------------------- PLOTS --------------------------

# --- ROS PUBLISHER
robot.publisher = create_rospublish(robot, 'robot_publisher')        

create_topic(robot.publisher, robot.device, 'state', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.base_estimator, 'q', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.imu_filters, 'imu_quat', robot = robot, data_type='vector')

create_topic(robot.publisher, robot.base_estimator, 'w_lf', robot = robot, data_type='double')
create_topic(robot.publisher, robot.base_estimator, 'w_rf', robot = robot, data_type='double')
create_topic(robot.publisher, robot.base_estimator, 'w_lf_filtered', robot = robot, data_type='double')
create_topic(robot.publisher, robot.base_estimator, 'w_rf_filtered', robot = robot, data_type='double')

create_topic(robot.publisher, robot.device, 'forceLLEG', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.device_filters.ft_LF_filter, 'x_filtered', robot = robot, data_type='vector')

create_topic(robot.publisher, robot.device, 'forceRLEG', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.device_filters.ft_RF_filter, 'x_filtered', robot = robot, data_type='vector')

create_topic(robot.publisher, robot.base_estimator, 'lf_xyzquat', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.base_estimator, 'rf_xyzquat', robot = robot, data_type='vector')

#create_topic(robot.publisher, robot.base_estimator, 'lf_ref_xyzquat', robot = robot, data_type='vector')
#create_topic(robot.publisher, robot.base_estimator, 'rf_ref_xyzquat', robot = robot, data_type='vector')