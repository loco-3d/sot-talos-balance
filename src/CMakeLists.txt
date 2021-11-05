# Copyright 2018, Gepetto team, LAAS-CNRS


SET(plugins
  example
  nmpc-online
  test-cython
  delay
  boolean-identity
  round-double-to-int
  int-identity
  pose-quaternion-to-matrix-homo
  euler-to-quat
  quat-to-euler
  state-transformation
  simple-reference-frame
  simple-state-integrator
  dummy-walking-pattern-generator
  simple-zmp-estimator
  simple-distribute-wrench
  simple-pid
  simple-pidd
  simple-controller-6d
  distribute-wrench
  dcm-com-controller
  dcm-controller
  dummy-dcm-estimator
  com-admittance-controller
  simple-admittance-controller
  coupled-admittance-controller
  admittance-controller-end-effector
  admittance-controller-op-point
  ankle-admittance-controller
  foot-force-difference-controller
  joint-position-controller
  nd-trajectory-generator
  talos-base-estimator
  talos-control-manager
  dcm-estimator
  qualisys-client
  ft-calibration
  ft-wrist-calibration
  ankle-joint-selector
  saturation
  hip-flexibility-compensation
  )

# set(feature-task_deps feature-generic task)

add_custom_command(OUTPUT
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/__init__.c
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/__init__${PYTHON_SOABI}
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/sum.c
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/sum${PYTHON_SOABI}
    ${CMAKE_CURRENT_SOURCE_DIR}/fooClass_api.h
    ${CMAKE_CURRENT_SOURCE_DIR}/fooClass.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/fooClass${PYTHON_SOABI}
    ${CMAKE_CURRENT_SOURCE_DIR}/fooClass.h
    COMMAND ${PYTHON_EXECUTABLE} setup.py build_ext --inplace
    DEPENDS
    libs/__init__.pxd
    libs/__init__.py
    libs/sum.pxd
    libs/sum.py
    fooClass.pyx
    setup.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/libs
    DESTINATION ${PYTHON_SITELIB}
    FILES_MATCHING REGEX ".*\.(so|py)$"
    )

set(nmpc_SOURCES
    fooClass_api.h
    fooClass.cpp
    embedded_main.cpp
    )

add_executable(nmpc ${nmpc_SOURCES})
target_include_directories(nmpc PRIVATE ${PYTHON_INCLUDE_DIRS} ${NUMPY_INCLUDE_DIRS})
target_link_libraries(nmpc PUBLIC ${PYTHON_LIBRARIES})
install(TARGETS nmpc EXPORT ${TARGETS_EXPORT_NAME} DESTINATION bin)

FOREACH(plugin ${plugins})
  GET_FILENAME_COMPONENT(LIBRARY_NAME ${plugin} NAME)
  ADD_LIBRARY(${LIBRARY_NAME} SHARED "${plugin}.cpp")

  IF(SUFFIX_SO_VERSION)
    SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
  ENDIF(SUFFIX_SO_VERSION)

  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} PUBLIC ${PROJECT_NAME} ${${LIBRARY_NAME}_deps})

  IF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
    INSTALL(TARGETS ${LIBRARY_NAME} EXPORT ${TARGETS_EXPORT_NAME}
      DESTINATION ${DYNAMIC_GRAPH_PLUGINDIR})
  ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)

  IF(BUILD_PYTHON_INTERFACE)
    STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${LIBRARY_NAME})
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python-module-py.cc")
      DYNAMIC_GRAPH_PYTHON_MODULE("${PY_NAME}/${PYTHON_LIBRARY_NAME}"
        ${LIBRARY_NAME} ${PROJECT_NAME}-${PYTHON_LIBRARY_NAME}-wrap
        SOURCE_PYTHON_MODULE "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python-module-py.cc")
    elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python.hh")
      DYNAMIC_GRAPH_PYTHON_MODULE("${PY_NAME}/${PYTHON_LIBRARY_NAME}"
        ${LIBRARY_NAME} ${PROJECT_NAME}-${PYTHON_LIBRARY_NAME}-wrap
        MODULE_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python.hh")
    endif()
  ENDIF(BUILD_PYTHON_INTERFACE)
ENDFOREACH(plugin)

target_include_directories(test-cython PRIVATE ${PYTHON_INCLUDE_DIRS} ${NUMPY_INCLUDE_DIRS})
target_link_libraries(test-cython PUBLIC ${PYTHON_LIBRARIES})

TARGET_LINK_LIBRARIES(distribute-wrench PUBLIC eiquadprog::eiquadprog)

# main python module
SET(${PROJECT_NAME}_PYTHON_FILES
  __init__.py
  main.py
  create_entities_utils.py
  meta_task_config.py
  meta_task_joint.py
  meta_task_pose.py
  motor_parameters.py
  )

# utils submodule
SET(${PROJECT_NAME}_UTILS_PYTHON_FILES
  __init__.py
  control_utils.py
  plot_utils.py
  sot_utils.py
  filter_utils.py
  run_test_utils.py
  gazebo_utils.py
  )

# talos submodule
SET(${PROJECT_NAME}_TALOS_PYTHON_FILES
  __init__.py
  admittance_ctrl_conf.py
  balance_ctrl_conf.py
  balance_ctrl_sim_conf.py
  base_estimator_conf.py
  base_estimator_sim_conf.py
  distribute_conf.py
  control_manager_conf.py
  control_manager_sim_conf.py
  current_controller_conf.py
  current_controller_sim_conf.py
  force_offset_conf.py
  force_torque_estimator_conf.py
  ft_calibration_conf.py
  ft_wrist_calibration_conf.py
  joint_torque_controller_conf.py
  joint_torque_controller_sim_conf.py
  parameter_server_conf.py
  hip_flexibility_compensation_conf.py
  )

# simulation files
SET(${PROJECT_NAME}_SIMULATION_FILES
  __init__.py
  test_dcm_zmp_control.py
  appli_dcm_zmp_control.py
  test_dcm_zmp_control_distribute.py
  appli_dcm_zmp_control_distribute.py
  test_dcm_zmp_control_ffdc.py
  test_dcm_zmp_control_bellStep.py
  appli_dcm_zmp_control_bellStep.py
  appli_dcm_zmp_control_ffdc.py
  test_dcm_zmp_control_flex.py
  appli_dcm_zmp_control_flex.py
  test_dcm_zmp_control_mlp.py
  appli_dcm_zmp_control_mlp.py
  test_dcm_zmp_control_flex_online.py
  appli_dcm_zmp_control_flex_online.py
  test_ffSubscriber.py
  test_ffSubscriber.py
  appli_ffSubscriber.py
  test_zmpEstimator.py
  appli_zmpEstimator.py
  test_dcmComControl.py
  appli_dcmComControl.py
  test_dcmComZmpControl.py
  appli_dcmComZmpControl.py
  test_dcmZmpControl.py
  appli_dcmZmpControl.py
  test_dcmZmpControl_file.py
  appli_dcmZmpControl_file.py
  test_comAdmittance.py
  test_admittance_single_joint.py
  appli_admittance_single_joint.py
  test_admittance_end_effector.py
  appli_admittance_end_effector.py
  test_admittance_drill.py
  appli_admittance_drill.py
  test_simple_ankle_admittance.py
  appli_simple_ankle_admittance.py
  test_admittance_single_joint_velocity_based.py
  appli_admittance_single_joint_velocity_based.py
  test_singleTraj.py
  test_jointTrajGen.py
  test_jointControl.py
  test_COMTraj.py
  appli_COMTraj.py
  test_COMTraj_tracer.py
  test_param_server.py
  test_dcm_estimator.py
  appli_dcm_estimator.py
  test_hip_flexibility_compensation.py
  appli_hip_flexibility_compensation.py
  test_admittance_drill.py
  appli_admittance_drill.py
  test_posture.py
  appli_posture.py
  )

# simulation files
SET(${PROJECT_NAME}_ROS_EXEC_FILES
  test_dcmZmpControl_file.py
  appli_dcmZmpControl_file.py
  )

IF(BUILD_PYTHON_INTERFACE)
  FOREACH(python ${${PROJECT_NAME}_PYTHON_FILES})
    PYTHON_INSTALL_ON_SITE(dynamic_graph/${PY_NAME} ${python})
  ENDFOREACH(python ${${PROJECT_NAME}_PYTHON_FILES})

  FOREACH(python ${${PROJECT_NAME}_TALOS_PYTHON_FILES})
    PYTHON_INSTALL_ON_SITE(dynamic_graph/${PY_NAME}/talos ${python})
  ENDFOREACH(python ${${PROJECT_NAME}_TALOS_PYTHON_FILES})

  FOREACH(python ${${PROJECT_NAME}_UTILS_PYTHON_FILES})
    PYTHON_INSTALL_ON_SITE(dynamic_graph/${PY_NAME}/utils ${python})
  ENDFOREACH(python ${${PROJECT_NAME}_UTILS_PYTHON_FILES})

  FOREACH(python ${${PROJECT_NAME}_SIMULATION_FILES})
    PYTHON_INSTALL_ON_SITE(dynamic_graph/${PY_NAME}/test ${python})
  ENDFOREACH(python ${${PROJECT_NAME}_SIMULATION_FILES})

  FOREACH(python ${${PROJECT_NAME}_ROS_EXEC_FILES})
    INSTALL(PROGRAMS dynamic_graph/${PY_NAME}/test/${python}
      DESTINATION lib/${PROJECT_NAME})
  ENDFOREACH(python ${${PROJECT_NAME}_ROS_EXEC_FILES})

ENDIF(BUILD_PYTHON_INTERFACE)
