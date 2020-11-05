# flake8: noqa
import math

import numpy as np
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT, FeaturePosture, Task

robot.timeStep = robot.device.getTimeStep()

# --- EXPERIMENTAL SET UP ------------------------------------------------------

device = 'simu'

# --- SET INITIAL CONFIGURATION ------------------------------------------------

q = [0., 0., 1.018213, 0., 0., 0.]  # Base
q += [0., 0., -0.411354, 0.859395, -0.448041, -0.001708]  # Left Leg
q += [0., 0., -0.411354, 0.859395, -0.448041, -0.001708]  # Right Leg
q += [0.0, 0.006761]  # Chest
q += [0.25847, 0.173046, -0.0002, -0.525366, 0., 0., 0.1, -0.005]  # Left Arm
q += [-0.25847, -0.173046, 0.0002, -0.525366, 0., 0., 0.1, -0.005]  # Right Arm
q += [0., 0.]  # Head
# robot.device.set(q)

# --- CREATE ENTITIES ----------------------------------------------------------

# --- POSTURE TASK -------------------------------------------------------------

robot.taskPosture = Task('task_posture')
robot.taskPosture.controlGain.value = 100.0
robot.taskPosture.feature = FeaturePosture('feature_posture')

q = list(robot.dynamic.position.value)
robot.taskPosture.feature.state.value = q
robot.taskPosture.feature.posture.value = q

robot.taskPosture.feature.selectDof(6, True)
robot.taskPosture.feature.selectDof(7, True)
robot.taskPosture.feature.selectDof(8, True)
robot.taskPosture.feature.selectDof(9, True)
robot.taskPosture.feature.selectDof(10, True)
robot.taskPosture.feature.selectDof(11, True)
robot.taskPosture.feature.selectDof(12, True)
robot.taskPosture.feature.selectDof(13, True)
robot.taskPosture.feature.selectDof(14, True)
robot.taskPosture.feature.selectDof(15, True)
robot.taskPosture.feature.selectDof(16, True)
robot.taskPosture.feature.selectDof(17, True)
robot.taskPosture.feature.selectDof(18, True)
robot.taskPosture.feature.selectDof(19, True)
robot.taskPosture.feature.selectDof(20, True)
robot.taskPosture.feature.selectDof(21, True)
robot.taskPosture.feature.selectDof(22, True)
robot.taskPosture.feature.selectDof(23, True)
robot.taskPosture.feature.selectDof(24, True)
robot.taskPosture.feature.selectDof(25, True)
robot.taskPosture.feature.selectDof(26, True)
robot.taskPosture.feature.selectDof(27, True)
robot.taskPosture.feature.selectDof(28, True)
robot.taskPosture.feature.selectDof(29, True)
robot.taskPosture.feature.selectDof(30, True)
robot.taskPosture.feature.selectDof(31, True)
robot.taskPosture.feature.selectDof(32, True)
robot.taskPosture.feature.selectDof(33, True)
robot.taskPosture.feature.selectDof(34, True)
robot.taskPosture.feature.selectDof(35, True)
robot.taskPosture.feature.selectDof(36, True)
robot.taskPosture.feature.selectDof(37, True)

robot.taskPosture.add(robot.taskPosture.feature.name)
plug(robot.dynamic.position, robot.taskPosture.feature.state)

robot.sot = SOT('sot')
robot.sot.setSize(robot.dynamic.getDimension())
plug(robot.sot.control, robot.device.control)

# --- PUSH THE TASKS -----------------------------------------------------------

robot.sot.push(robot.taskPosture.name)
