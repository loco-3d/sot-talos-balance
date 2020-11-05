# flake8: noqa
from math import sqrt

import numpy as np
from rospkg import RosPack
import sot_talos_balance.talos.base_estimator_conf as base_estimator_conf
import sot_talos_balance.talos.control_manager_conf as cm_conf
import sot_talos_balance.talos.ft_calibration_conf as ft_conf
import sot_talos_balance.talos.hip_flexibility_compensation_conf as hipFlexCompConfig
import sot_talos_balance.talos.parameter_server_conf as param_server_conf
from dynamic_graph import plug
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Vector
from dynamic_graph.sot.core.feature_posture import FeaturePosture
from dynamic_graph.sot.core.operator import MatrixHomoToPoseQuaternion
from dynamic_graph.sot.core.sot import SOT, Task
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph.sot.dynamic_pinocchio import DynamicPinocchio
from dynamic_graph.tracer_real_time import TracerRealTime
from sot_talos_balance.create_entities_utils import *
from dynamic_graph.sot.pattern_generator import PatternGenerator

cm_conf.CTRL_MAX = 100.0  # temporary hack

robot.timeStep = robot.device.getTimeStep()
dt = robot.timeStep
robot.device.setControlInputType("noInteg")

# --- Pendulum parameters
robot_name = 'robot'
robot.dynamic.com.recompute(0)
robotDim = robot.dynamic.getDimension()
mass = robot.dynamic.data.mass[0]
h = robot.dynamic.com.value[2]
g = 9.81
omega = sqrt(g / h)

# --- Parameter server
fill_parameter_server(robot.param_server,param_server_conf, dt)

# --- Initial feet and waist
robot.dynamic.createOpPoint('LF', robot.OperationalPointsMap['left-ankle'])
robot.dynamic.createOpPoint('RF', robot.OperationalPointsMap['right-ankle'])
robot.dynamic.createOpPoint('WT', robot.OperationalPointsMap['waist'])
robot.dynamic.LF.recompute(0)
robot.dynamic.RF.recompute(0)
robot.dynamic.WT.recompute(0)
robot.dynamic.position.recompute(0)
robot.dynamic.com.recompute(0)
# -------------------------- DESIRED TRAJECTORY --------------------------

rospack = RosPack()  

# -------------------------- PATTERN GENERATOR --------------------------

robot.pg = PatternGenerator('pg')

# MODIFIED WITH MY PATHS
talos_data_folder = rospack.get_path('talos_data')
# robot.pg.setURDFpath(talos_data_folder + '/urdf/talos_reduced_v2.urdf')
robot.pg.setSRDFpath(talos_data_folder + '/srdf/talos_wpg.srdf')
## END MODIFIED

robot.pg.buildModel()

robot.pg.parseCmd(":samplingperiod 0.005")
robot.pg.parseCmd(":previewcontroltime 1.6")
robot.pg.parseCmd(":omega 0.0")
robot.pg.parseCmd(':stepheight 0.05')
robot.pg.parseCmd(':doublesupporttime 0.2')
robot.pg.parseCmd(':singlesupporttime 1.0')
robot.pg.parseCmd(":armparameters 0.5")
robot.pg.parseCmd(":LimitsFeasibility 0.0")
robot.pg.parseCmd(":ZMPShiftParameters 0.015 0.015 0.015 0.015")
robot.pg.parseCmd(":TimeDistributeParameters 2.0 3.5 1.7 3.0")
robot.pg.parseCmd(":UpperBodyMotionParameters -0.1 -1.0 0.0")
robot.pg.parseCmd(":comheight 0.876681") #0.8926753
robot.pg.parseCmd(":setVelReference  0.1 0.0 0.0")

plug(robot.dynamic.position, robot.pg.position)
plug(robot.dynamic.com, robot.pg.com)
plug(robot.dynamic.LF, robot.pg.leftfootcurrentpos)
plug(robot.dynamic.RF, robot.pg.rightfootcurrentpos)
robotDim = len(robot.dynamic.velocity.value)
robot.pg.motorcontrol.value = robotDim * (0, )
robot.pg.zmppreviouscontroller.value = (0, 0, 0)

robot.pg.initState()

# -------------------------- TRIGGER --------------------------

robot.triggerPG = BooleanIdentity('triggerPG')
robot.triggerPG.sin.value = 0
plug(robot.triggerPG.sout, robot.pg.trigger)


# --- Interface with controller entities

wp = DummyWalkingPatternGenerator('dummy_wp')
wp.init()
wp.omega.value = omega
plug(robot.pg.waistattitudematrixabsolute, wp.waist)
plug(robot.pg.leftfootref, wp.footLeft)
plug(robot.pg.rightfootref, wp.footRight)
plug(robot.pg.comref, wp.com)
plug(robot.pg.dcomref, wp.vcom)
plug(robot.pg.ddcomref, wp.acom)

robot.wp = wp

# --- Compute the values to use them in initialization
robot.wp.comDes.recompute(0)
robot.wp.dcmDes.recompute(0)
robot.wp.zmpDes.recompute(0)

# -------------------------- ESTIMATION --------------------------

# --- Base Estimation
robot.device_filters = create_device_filters(robot, dt)
robot.imu_filters = create_imu_filters(robot, dt)
robot.base_estimator = create_base_estimator(robot, dt, base_estimator_conf)

robot.m2qLF = MatrixHomoToPoseQuaternion('m2qLF')
plug(robot.dynamic.LF, robot.m2qLF.sin)
plug(robot.m2qLF.sout, robot.base_estimator.lf_ref_xyzquat)
robot.m2qRF = MatrixHomoToPoseQuaternion('m2qRF')
plug(robot.dynamic.RF, robot.m2qRF.sin)
plug(robot.m2qRF.sout, robot.base_estimator.rf_ref_xyzquat)

# robot.be_filters              = create_be_filters(robot, dt)

# --- Conversion
e2q = EulerToQuat('e2q')
plug(robot.base_estimator.q, e2q.euler)
robot.e2q = e2q

# --- Kinematic computations
robot.rdynamic = DynamicPinocchio("real_dynamics")
robot.rdynamic.setModel(robot.dynamic.model)
robot.rdynamic.setData(robot.rdynamic.model.createData())
plug(robot.base_estimator.q, robot.rdynamic.position)
robot.rdynamic.velocity.value = [0.0] * robotDim
robot.rdynamic.acceleration.value = [0.0] * robotDim

# --- CoM Estimation
cdc_estimator = DcmEstimator('cdc_estimator')
cdc_estimator.init(dt, robot_name)
plug(robot.e2q.quaternion, cdc_estimator.q)
plug(robot.base_estimator.v, cdc_estimator.v)
robot.cdc_estimator = cdc_estimator

# --- DCM Estimation
estimator = DummyDcmEstimator("dummy")
plug(robot.wp.omegaDes, estimator.omega)
estimator.mass.value = 1.0
plug(robot.cdc_estimator.c, estimator.com)
plug(robot.cdc_estimator.dc, estimator.momenta)
estimator.init()
robot.estimator = estimator

# --- Force calibration
robot.ftc = create_ft_calibrator(robot, ft_conf)

# --- ZMP estimation
zmp_estimator = SimpleZmpEstimator("zmpEst")
robot.rdynamic.createOpPoint('sole_LF', 'left_sole_link')
robot.rdynamic.createOpPoint('sole_RF', 'right_sole_link')
plug(robot.rdynamic.sole_LF, zmp_estimator.poseLeft)
plug(robot.rdynamic.sole_RF, zmp_estimator.poseRight)
plug(robot.ftc.left_foot_force_out, zmp_estimator.wrenchLeft)
plug(robot.ftc.right_foot_force_out, zmp_estimator.wrenchRight)
zmp_estimator.init()
robot.zmp_estimator = zmp_estimator

# -------------------------- ADMITTANCE CONTROL --------------------------

# --- DCM controller
Kp_dcm = [8.0] * 3
Ki_dcm = [0.0, 0.0, 0.0]  # zero (to be set later)
Kz_dcm = [0.] * 3
gamma_dcm = 0.2

dcm_controller = DcmController("dcmCtrl")

dcm_controller.Kp.value = Kp_dcm
dcm_controller.Ki.value = Ki_dcm
dcm_controller.Kz.value = Kz_dcm
dcm_controller.decayFactor.value = gamma_dcm
dcm_controller.mass.value = mass
plug(robot.wp.omegaDes, dcm_controller.omega)

plug(robot.cdc_estimator.c, dcm_controller.com)
plug(robot.estimator.dcm, dcm_controller.dcm)

plug(robot.wp.zmpDes, dcm_controller.zmpDes)
plug(robot.wp.dcmDes, dcm_controller.dcmDes)

plug(robot.zmp_estimator.zmp, dcm_controller.zmp)

dcm_controller.init(dt)

robot.dcm_control = dcm_controller

Ki_dcm = [1.0, 1.0, 1.0]  # this value is employed later

Kz_dcm = [0.0, 0.0, 0.0]  # this value is employed later

# --- CoM admittance controller
Kp_adm = [0.0, 0.0, 0.0]  # zero (to be set later)

com_admittance_control = ComAdmittanceController("comAdmCtrl")
com_admittance_control.Kp.value = Kp_adm
plug(robot.zmp_estimator.zmp, com_admittance_control.zmp)
com_admittance_control.zmpDes.value = robot.wp.zmpDes.value  # should be plugged to robot.dcm_control.zmpRef
plug(robot.wp.acomDes, com_admittance_control.ddcomDes)

com_admittance_control.init(dt)
com_admittance_control.setState(robot.wp.comDes.value, [0.0, 0.0, 0.0])

robot.com_admittance_control = com_admittance_control

Kp_adm = [15.0, 15.0, 0.0]  # this value is employed later

# --- Control Manager
robot.cm = create_ctrl_manager(cm_conf, dt, robot_name='robot')
robot.cm.addCtrlMode('sot_input')
robot.cm.setCtrlMode('all', 'sot_input')
robot.cm.addEmergencyStopSIN('zmp')

# -------------------------- SOT CONTROL --------------------------

# --- Upper body
robot.taskUpperBody = Task('task_upper_body')
robot.taskUpperBody.feature = FeaturePosture('feature_upper_body')

q = list(robot.dynamic.position.value)
robot.taskUpperBody.feature.state.value = q
robot.taskUpperBody.feature.posture.value = q

robotDim = robot.dynamic.getDimension()  # 38
for i in range(18, robotDim):
    robot.taskUpperBody.feature.selectDof(i, True)

robot.taskUpperBody.controlGain.value = 100.0
robot.taskUpperBody.add(robot.taskUpperBody.feature.name)
plug(robot.dynamic.position, robot.taskUpperBody.feature.state)

# --- CONTACTS
#define contactLF and contactRF
robot.contactLF = MetaTaskKine6d('contactLF', robot.dynamic, 'LF', robot.OperationalPointsMap['left-ankle'])
robot.contactLF.feature.frame('desired')
robot.contactLF.gain.setConstant(300)
plug(robot.wp.footLeftDes, robot.contactLF.featureDes.position)  #.errorIN?
locals()['contactLF'] = robot.contactLF

robot.contactRF = MetaTaskKine6d('contactRF', robot.dynamic, 'RF', robot.OperationalPointsMap['right-ankle'])
robot.contactRF.feature.frame('desired')
robot.contactRF.gain.setConstant(300)
plug(robot.wp.footRightDes, robot.contactRF.featureDes.position)  #.errorIN?
locals()['contactRF'] = robot.contactRF

# --- COM height
robot.taskComH = MetaTaskKineCom(robot.dynamic, name='comH')
plug(robot.wp.comDes, robot.taskComH.featureDes.errorIN)
robot.taskComH.task.controlGain.value = 2.
robot.taskComH.feature.selec.value = '100'

# --- COM
robot.taskCom = MetaTaskKineCom(robot.dynamic)
plug(robot.com_admittance_control.comRef, robot.taskCom.featureDes.errorIN)
plug(robot.com_admittance_control.dcomRef, robot.taskCom.featureDes.errordotIN)
robot.taskCom.task.controlGain.value = 100.
robot.taskCom.task.setWithDerivative(True)
robot.taskCom.feature.selec.value = '011'

# --- Waist
robot.keepWaist = MetaTaskKine6d('keepWaist', robot.dynamic, 'WT', robot.OperationalPointsMap['waist'])
robot.keepWaist.feature.frame('desired')
robot.keepWaist.gain.setConstant(300)
plug(robot.wp.waistDes, robot.keepWaist.featureDes.position)
robot.keepWaist.feature.selec.value = '111000'
locals()['keepWaist'] = robot.keepWaist

# --- SOT solver
robot.sot = SOT('sot')
robot.sot.setSize(robot.dynamic.getDimension())

# --- State integrator
robot.integrate = SimpleStateIntegrator("integrate")
robot.integrate.init(dt)
robot.integrate.setState(robot.device.state.value)
robot.integrate.setVelocity(robot.dynamic.getDimension() * [0.])

# --- Hip flexibility compensation --------------------------------

robot.hipComp = create_hip_flexibility_compensation(robot, hipFlexCompConfig, robot_name)
plug(robot.pg.contactphase, robot.hipComp.phase)
if not flexi:
    robot.hipComp.K_l.value = float('inf')  #disable
    robot.hipComp.K_r.value = float('inf')  #disable

# --- Plug SOT control to device through control manager, state integrator and hip flexibility compensation
plug(robot.sot.control, robot.cm.ctrl_sot_input)
plug(robot.cm.u_safe, robot.integrate.control)
plug(robot.integrate.state, robot.hipComp.q_des)
plug(robot.hipComp.q_cmd, robot.device.control)

robot.sot.push(robot.taskUpperBody.name)
robot.sot.push(robot.contactRF.task.name)
robot.sot.push(robot.contactLF.task.name)
robot.sot.push(robot.taskComH.task.name)
robot.sot.push(robot.taskCom.task.name)
robot.sot.push(robot.keepWaist.task.name)

# --- Delay
robot.delay_pos = DelayVector("delay_pos")
robot.delay_pos.setMemory(robot.device.state.value)
robot.device.before.addSignal(robot.delay_pos.name + '.current')
plug(robot.integrate.state, robot.delay_pos.sin)

robot.delay_vel = DelayVector("delay_vel")
robot.delay_vel.setMemory(robot.dynamic.getDimension() * [0.])
robot.device.before.addSignal(robot.delay_vel.name + '.current')
plug(robot.cm.u_safe, robot.delay_vel.sin)

# --- Plug integrator instead of device
plug(robot.delay_vel.previous, robot.vselec.sin)

# Remove flexibility compensation of the encoders position for base estimator
plug(robot.device.joint_angles, robot.hipComp.q_enc)
plug(robot.hipComp.q_cmd_enc, robot.base_estimator.joint_positions)

# --- Fix robot.dynamic inputs
plug(robot.delay_pos.previous, robot.dynamic.position)
plug(robot.delay_vel.previous, robot.dynamic.velocity)
robot.dvdt = Derivator_of_Vector("dv_dt")
robot.dvdt.dt.value = dt
plug(robot.delay_vel.previous, robot.dvdt.sin)
plug(robot.dvdt.sout, robot.dynamic.acceleration)

# -------------------------- PLOTS --------------------------

# --- ROS PUBLISHER
robot.publisher = create_rospublish(robot, 'robot_publisher')

create_topic(robot.publisher, robot.device, 'state', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.base_estimator, 'q', robot=robot, data_type='vector')
#create_topic(robot.publisher, robot.stf, 'q', robot = robot, data_type='vector')

create_topic(robot.publisher, robot.pg, 'contactphase', robot = robot, data_type='int')

create_topic(robot.publisher, robot.pg, 'comref', robot=robot, data_type='vector')  # generated CoM
create_topic(robot.publisher, robot.pg, 'dcomref', robot=robot, data_type='vector')  # generated CoM velocity
create_topic(robot.publisher, robot.pg, 'ddcomref', robot=robot, data_type='vector')  # generated CoM acceleration

create_topic(robot.publisher, robot.wp, 'comDes', robot=robot, data_type='vector')  # desired CoM

create_topic(robot.publisher, robot.cdc_estimator, 'c', robot=robot, data_type='vector')  # estimated CoM
create_topic(robot.publisher, robot.cdc_estimator, 'dc', robot=robot, data_type='vector')  # estimated CoM velocity

create_topic(robot.publisher, robot.com_admittance_control, 'comRef', robot=robot, data_type='vector')  # reference CoM
create_topic(robot.publisher, robot.dynamic, 'com', robot=robot, data_type='vector')  # resulting SOT CoM

create_topic(robot.publisher, robot.dcm_control, 'dcmDes', robot=robot, data_type='vector')  # desired DCM
create_topic(robot.publisher, robot.estimator, 'dcm', robot=robot, data_type='vector')  # estimated DCM

# create_topic(robot.publisher, robot.zmpTrajGen, 'x', robot=robot, data_type='vector')  # generated ZMP
create_topic(robot.publisher, robot.wp, 'zmpDes', robot=robot, data_type='vector')  # desired ZMP
create_topic(robot.publisher, robot.dynamic, 'zmp', robot=robot, data_type='vector')  # SOT ZMP
create_topic(robot.publisher, robot.zmp_estimator, 'zmp', robot=robot, data_type='vector')  # estimated ZMP
create_topic(robot.publisher, robot.dcm_control, 'zmpRef', robot=robot, data_type='vector')  # reference ZMP

create_topic(robot.publisher, robot.m2qLF, 'sout', robot=robot, data_type='vector')  # reference ZMP
create_topic(robot.publisher, robot.m2qRF, 'sout', robot=robot, data_type='vector')  # reference ZMP
#create_topic(robot.publisher, robot.device, 'forceLLEG', robot = robot, data_type='vector')               # measured left wrench
#create_topic(robot.publisher, robot.device, 'forceRLEG', robot = robot, data_type='vector')               # measured right wrench

#create_topic(robot.publisher, robot.device_filters.ft_LF_filter, 'x_filtered', robot = robot, data_type='vector') # filtered left wrench
#create_topic(robot.publisher, robot.device_filters.ft_RF_filter, 'x_filtered', robot = robot, data_type='vector') # filtered right wrench

create_topic(robot.publisher, robot.pg, 'waistattitudematrixabsolute', robot=robot, data_type='matrixHomo')  # desired waist orientation

create_topic(robot.publisher, robot.pg, 'leftfootref', robot=robot, data_type='matrixHomo')  # desired left foot pose
create_topic(robot.publisher, robot.pg, 'rightfootref', robot=robot, data_type='matrixHomo')  # desired right foot pose

create_topic(robot.publisher, robot.ftc, 'left_foot_force_out', robot=robot, data_type='vector')  # calibrated left wrench
create_topic(robot.publisher, robot.ftc, 'right_foot_force_out', robot=robot, data_type='vector')  # calibrated right wrench

create_topic(robot.publisher, robot.device, 'forceLLEG', robot = robot, data_type='vector') # measured left wrench
create_topic(robot.publisher, robot.device, 'forceRLEG', robot = robot, data_type='vector')

create_topic(robot.publisher, robot.dynamic, 'LF', robot=robot, data_type='matrixHomo')  # left foot
create_topic(robot.publisher, robot.dynamic, 'RF', robot=robot, data_type='matrixHomo')  # right foot

create_topic(robot.publisher, robot.zmp_estimator, 'poseLeft', robot=robot, data_type='matrixHomo')  
create_topic(robot.publisher, robot.zmp_estimator, 'poseRight', robot=robot, data_type='matrixHomo') 
create_topic(robot.publisher, robot.device, 'ptorque', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.hipComp, 'delta_q', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.hipComp, 'q_cmd', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.hipComp, 'q_des', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.hipComp, 'tau', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.hipComp, 'tau_filt', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.hipComp, 'q_enc', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.hipComp, 'q_cmd_enc', robot=robot, data_type='vector')

# --- TRACER
robot.tracer = TracerRealTime("com_tracer")
robot.tracer.setBufferSize(80*(2**20))
robot.tracer.open('/tmp','dg_','.dat')
robot.device.after.addSignal('{0}.triger'.format(robot.tracer.name))

addTrace(robot.tracer, robot.wp, 'comDes')                      # desired CoM

addTrace(robot.tracer, robot.cdc_estimator, 'c')                # estimated CoM
#addTrace(robot.tracer, robot.cdc_estimator, 'dc')               # estimated CoM velocity

addTrace(robot.tracer, robot.com_admittance_control, 'comRef')  # reference CoM
addTrace(robot.tracer, robot.dynamic, 'com')                    # resulting SOT CoM

#addTrace(robot.tracer, robot.dcm_control, 'dcmDes')             # desired DCM
#addTrace(robot.tracer, robot.estimator, 'dcm')                  # estimated DCM

#addTrace(robot.tracer, robot.dcm_control, 'zmpDes')             # desired ZMP
#addTrace(robot.tracer, robot.dynamic, 'zmp')                    # SOT ZMP
addTrace(robot.tracer, robot.zmp_estimator, 'zmp')              # estimated ZMP
addTrace(robot.tracer, robot.dcm_control, 'zmpRef')             # reference ZMP

#addTrace(robot.tracer, robot.ftc, 'left_foot_force_out')        # calibrated left wrench
#addTrace(robot.tracer,  robot.ftc, 'right_foot_force_out')      # calibrated right wrench

#addTrace(robot.tracer,  robot.dynamic, 'LF')                    # left foot
#addTrace(robot.tracer,  robot.dynamic, 'RF')                    # right foot

robot.tracer.start()

