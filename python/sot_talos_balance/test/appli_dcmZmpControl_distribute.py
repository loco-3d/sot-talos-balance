from sot_talos_balance.create_entities_utils import *
import sot_talos_balance.talos.parameter_server_conf   as param_server_conf
import sot_talos_balance.talos.base_estimator_conf     as base_estimator_conf
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph.sot.core import Task, FeaturePosture
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT
from math import sqrt

from dynamic_graph.tracer_real_time import TracerRealTime

from dynamic_graph.sot.dynamics_pinocchio import DynamicPinocchio

robot.timeStep = robot.device.getTimeStep()
dt = robot.timeStep;

# -------------------------- DESIRED TRAJECTORY --------------------------

# --- Desired values

# --- Desired CoM
robot.dynamic.com.recompute(0)
comDes = robot.dynamic.com.value

## --- Translation
#comDes = list(comDes)
#comDes[0] += 0.01
#comDes[1] += 0.01
#comDes = tuple(comDes)

# --- Conversion to center of feet reference frame
import pinocchio as pin
import numpy as np
model = robot.dynamic.model
data = robot.dynamic.data # DO NOT make any computation with this data object. Kinematic computation have already been done
leftName = param_server_conf.footFrameNames['Left']
leftId = model.getFrameId(leftName)
leftPos  = data.oMf[leftId]
rightName = param_server_conf.footFrameNames['Right']
rightId = model.getFrameId(rightName)
rightPos = data.oMf[rightId]
centerTranslation = ( data.oMf[rightId].translation + data.oMf[leftId].translation )/2 + np.matrix(param_server_conf.rightFootSoleXYZ).T
centerPos = pin.SE3(rightPos.rotation,centerTranslation)

comDes = centerPos.actInv(np.matrix(comDes).T)
comDes = tuple(comDes.flatten().tolist()[0])

# --- Desired DCM and ZMP
dcmDes = comDes
zmpDes = comDes[:2] + (0.0,)

# --- Desired CoM acceleration
ddcomDes = (0.0,0.0,0.0)

# --- Pendulum parameters
robot_name='robot'
robotDim = robot.dynamic.getDimension()
mass = robot.dynamic.data.mass[0]
h = robot.dynamic.com.value[2]
g = 9.81
omega = sqrt(g/h)

# -------------------------- ESTIMATION --------------------------

# --- Base Estimation
robot.param_server            = create_parameter_server(param_server_conf,dt)
robot.device_filters          = create_device_filters(robot, dt)
robot.imu_filters             = create_imu_filters(robot, dt)
robot.base_estimator          = create_base_estimator(robot, dt, base_estimator_conf)
# robot.be_filters              = create_be_filters(robot, dt)

# --- Conversion
e2q = EulerToQuat('e2q')
plug(robot.base_estimator.q,e2q.euler)
robot.e2q = e2q

# --- Kinematic computations
robot.rdynamic = DynamicPinocchio("real_dynamics")
robot.rdynamic.setModel(robot.dynamic.model)
robot.rdynamic.setData(robot.rdynamic.model.createData())
plug(robot.base_estimator.q,robot.rdynamic.position)
robot.rdynamic.velocity.value = [0.0]*robotDim
robot.rdynamic.acceleration.value = [0.0]*robotDim

# --- CoM Estimation
cdc_estimator = DcmEstimator('cdc_estimator')
cdc_estimator.init(dt, robot_name)
plug(robot.e2q.quaternion, cdc_estimator.q)
plug(robot.base_estimator.v, cdc_estimator.v)
robot.cdc_estimator = cdc_estimator

# --- DCM Estimation
estimator = DummyDcmEstimator("dummy")
estimator.omega.value = omega
estimator.mass.value = 1.0
plug(robot.cdc_estimator.c, estimator.com)
plug(robot.cdc_estimator.dc,estimator.momenta)
estimator.init()
robot.estimator = estimator

# --- ZMP estimation
zmp_estimator = SimpleZmpEstimator("zmpEst")
robot.rdynamic.createOpPoint('sole_LF','left_sole_link')
robot.rdynamic.createOpPoint('sole_RF','right_sole_link')
plug(robot.rdynamic.sole_LF,zmp_estimator.poseLeft)
plug(robot.rdynamic.sole_RF,zmp_estimator.poseRight)
plug(robot.device_filters.ft_LF_filter.x_filtered,zmp_estimator.wrenchLeft)
plug(robot.device_filters.ft_RF_filter.x_filtered,zmp_estimator.wrenchRight)
zmp_estimator.init()
robot.zmp_estimator = zmp_estimator

# -------------------------- ADMITTANCE CONTROL --------------------------

# --- DCM controller
Kp_dcm = [5.0,5.0,5.0]
Ki_dcm = [0.0,0.0,0.0] # zero (to be set later)
gamma_dcm = 0.2

dcm_controller = DcmController("dcmCtrl")

dcm_controller.Kp.value = Kp_dcm
dcm_controller.Ki.value = Ki_dcm
dcm_controller.decayFactor.value = gamma_dcm
dcm_controller.mass.value = mass
dcm_controller.omega.value = omega

plug(robot.cdc_estimator.c,dcm_controller.com)
plug(robot.estimator.dcm,dcm_controller.dcm)

dcm_controller.zmpDes.value = zmpDes # plug a signal here
dcm_controller.dcmDes.value = dcmDes # plug a signal here

dcm_controller.init(dt)

robot.dcm_control = dcm_controller

Ki_dcm = [0.1,0.1,0.1] # this value is employed later

# --- Distribute wrench
distribute = DistributeWrench('distribute')
plug(robot.e2q.quaternion, distribute.q)
plug(robot.dcm_control.wrenchRef, distribute.wrenchDes)
distribute.init(robot_name)
robot.distribute = distribute

# --- CoM admittance controller
Kp_adm = [0.0,0.0,0.0] # zero (to be set later)

com_admittance_control = ComAdmittanceController("comAdmCtrl")
com_admittance_control.Kp.value = Kp_adm
plug(robot.zmp_estimator.zmp,com_admittance_control.zmp)
com_admittance_control.zmpDes.value = zmpDes     # should be plugged to robot.distribute.zmpRef
com_admittance_control.ddcomDes.value = ddcomDes # plug a signal here

com_admittance_control.init(dt)
com_admittance_control.setState(comDes,[0.0,0.0,0.0])

robot.com_admittance_control = com_admittance_control

Kp_adm = [0.1,0.1,0.0] # this value is employed later

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
robot.contactLF.keep()
locals()['contactLF'] = robot.contactLF

robot.contactRF = MetaTaskKine6d('contactRF',robot.dynamic,'RF',robot.OperationalPointsMap['right-ankle'])
robot.contactRF.feature.frame('desired')
robot.contactRF.gain.setConstant(300)
robot.contactRF.keep()
locals()['contactRF'] = robot.contactRF

# --- COM
robot.taskCom = MetaTaskKineCom(robot.dynamic)
plug(robot.com_admittance_control.comRef,robot.taskCom.featureDes.errorIN)
plug(robot.com_admittance_control.dcomRef,robot.taskCom.featureDes.errordotIN)
robot.taskCom.task.controlGain.value = 0
robot.taskCom.task.setWithDerivative(True)

# --- Waist
robot.keepWaist = MetaTaskKine6d('keepWaist',robot.dynamic,'WT',robot.OperationalPointsMap['waist'])
robot.keepWaist.feature.frame('desired')
robot.keepWaist.gain.setConstant(300)
robot.keepWaist.keep()
robot.keepWaist.feature.selec.value = '111000'
locals()['keepWaist'] = robot.keepWaist

# # --- Posture
# robot.taskPos = Task ('task_pos')
# robot.taskPos.feature = FeaturePosture('feature_pos')
#
# q = list(robot.dynamic.position.value)
# robot.taskPos.feature.state.value = q
# robot.taskPos.feature.posture.value = q

# robotDim = robot.dynamic.getDimension() # 38
#robot.taskPos.feature.selectDof(6,True)
#robot.taskPos.feature.selectDof(7,True)
#robot.taskPos.feature.selectDof(8,True)
#robot.taskPos.feature.selectDof(9,True)
#robot.taskPos.feature.selectDof(10,True)
#robot.taskPos.feature.selectDof(11,True)
#robot.taskPos.feature.selectDof(12,True)
#robot.taskPos.feature.selectDof(13,True)
#robot.taskPos.feature.selectDof(14,True)
#robot.taskPos.feature.selectDof(15,True)
#robot.taskPos.feature.selectDof(16,True)
#robot.taskPos.feature.selectDof(17,True)
#robot.taskPos.feature.selectDof(18,True)
#robot.taskPos.feature.selectDof(19,True)
#robot.taskPos.feature.selectDof(20,True)
#robot.taskPos.feature.selectDof(21,True)
#robot.taskPos.feature.selectDof(22,True)
#robot.taskPos.feature.selectDof(23,True)
#robot.taskPos.feature.selectDof(24,True)
#robot.taskPos.feature.selectDof(25,True)
#robot.taskPos.feature.selectDof(26,True)
#robot.taskPos.feature.selectDof(27,True)
#robot.taskPos.feature.selectDof(28,True)
#robot.taskPos.feature.selectDof(29,True)
#robot.taskPos.feature.selectDof(30,True)
#robot.taskPos.feature.selectDof(31,True)
#robot.taskPos.feature.selectDof(32,True)
#robot.taskPos.feature.selectDof(33,True)
#robot.taskPos.feature.selectDof(34,True)
#robot.taskPos.feature.selectDof(35,True)
#robot.taskPos.feature.selectDof(36,True)
#robot.taskPos.feature.selectDof(37,True)

#robot.taskPos.controlGain.value = 100.0
#robot.taskPos.add(robot.taskPos.feature.name)
#plug(robot.dynamic.position, robot.taskPos.feature.state)

# --- SOT solver
robot.sot = SOT('sot')
robot.sot.setSize(robot.dynamic.getDimension())
plug(robot.sot.control,robot.device.control)

robot.sot.push(robot.taskUpperBody.name)
robot.sot.push(robot.contactRF.task.name)
robot.sot.push(robot.contactLF.task.name)
robot.sot.push(robot.taskCom.task.name)
robot.sot.push(robot.keepWaist.task.name)
# robot.sot.push(robot.taskPos.name)
# robot.device.control.recompute(0) # this crashes as it employs joint sensors which are not ready yet

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

rospub_signalName = '{0}_{1}'.format('fake', 'comDes')
topicname = '/sot/{0}/{1}'.format('fake', 'comDes')
robot.publisher.add('vector',rospub_signalName,topicname)
plug(robot.dcm_control.dcmDes, robot.publisher.signal(rospub_signalName))                                 # desired CoM (workaround)

create_topic(robot.publisher, robot.cdc_estimator, 'c', robot = robot, data_type='vector')                # estimated CoM
create_topic(robot.publisher, robot.cdc_estimator, 'dc', robot = robot, data_type='vector')               # estimated CoM velocity

create_topic(robot.publisher, robot.com_admittance_control, 'comRef', robot = robot, data_type='vector')  # reference CoM
create_topic(robot.publisher, robot.dynamic, 'com', robot = robot, data_type='vector')                    # resulting SOT CoM

create_topic(robot.publisher, robot.dcm_control, 'dcmDes', robot = robot, data_type='vector')             # desired DCM
create_topic(robot.publisher, robot.estimator, 'dcm', robot = robot, data_type='vector')                  # estimated DCM

create_topic(robot.publisher, robot.dcm_control, 'zmpDes', robot = robot, data_type='vector')             # desired ZMP
create_topic(robot.publisher, robot.dynamic, 'zmp', robot = robot, data_type='vector')                    # SOT ZMP
create_topic(robot.publisher, robot.zmp_estimator, 'zmp', robot = robot, data_type='vector')              # estimated ZMP
create_topic(robot.publisher, robot.dcm_control, 'zmpRef', robot = robot, data_type='vector')             # unoptimized reference ZMP

create_topic(robot.publisher, robot.dcm_control, 'wrenchRef', robot = robot, data_type='vector')          # unoptimized reference wrench
create_topic(robot.publisher, robot.distribute, 'wrenchLeft', robot = robot, data_type='vector')          # reference left wrench
create_topic(robot.publisher, robot.distribute, 'wrenchRight', robot = robot, data_type='vector')         # reference right wrench
create_topic(robot.publisher, robot.distribute, 'wrenchRef', robot = robot, data_type='vector')           # optimized reference wrench

# --- TRACER
robot.tracer = TracerRealTime("zmp_tracer")
robot.tracer.setBufferSize(80*(2**20))
robot.tracer.open('/tmp','dg_','.dat')
robot.device.after.addSignal('{0}.triger'.format(robot.tracer.name))

addTrace(robot.tracer, robot.dcm_control, 'dcmDes')             # desired CoM (workaround)
addTrace(robot.tracer, robot.cdc_estimator, 'c')                # estimated CoM
addTrace(robot.tracer, robot.com_admittance_control, 'comRef')  # reference CoM
addTrace(robot.tracer, robot.dynamic, 'com')                    # resulting SOT CoM

# addTrace(robot.tracer, robot.dcm_control, 'dcmDes')           # desired DCM (already added)
addTrace(robot.tracer, robot.estimator, 'dcm')                  # estimated DCM

addTrace(robot.tracer, robot.dcm_control, 'zmpDes')             # desired ZMP
addTrace(robot.tracer, robot.zmp_estimator, 'zmp')              # estimated ZMP
addTrace(robot.tracer, robot.dcm_control, 'zmpRef')             # reference ZMP
addTrace(robot.tracer, robot.dynamic, 'zmp')                    # SOT ZMP

# -------------------------- SIMULATION --------------------------

robot.tracer.start()
