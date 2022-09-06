# Copyright 2018, Gepetto team, LAAS-CNRS

set(plugins
    example
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
    hip-flexibility-compensation)

# set(feature-task_deps feature-generic task)

foreach(plugin ${plugins})
  get_filename_component(LIBRARY_NAME ${plugin} NAME)
  add_library(${LIBRARY_NAME} SHARED "${plugin}.cpp")

  if(SUFFIX_SO_VERSION)
    set_target_properties(${LIBRARY_NAME} PROPERTIES SOVERSION
                                                     ${PROJECT_VERSION})
  endif(SUFFIX_SO_VERSION)

  target_link_libraries(${LIBRARY_NAME} PUBLIC ${PROJECT_NAME}
                                               ${${LIBRARY_NAME}_deps})

  if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
    install(
      TARGETS ${LIBRARY_NAME}
      EXPORT ${TARGETS_EXPORT_NAME}
      DESTINATION ${DYNAMIC_GRAPH_PLUGINDIR})
  endif(NOT INSTALL_PYTHON_INTERFACE_ONLY)

  if(BUILD_PYTHON_INTERFACE)
    string(REPLACE - _ PYTHON_LIBRARY_NAME ${LIBRARY_NAME})
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python-module-py.cc")
      dynamic_graph_python_module(
        "${PY_NAME}/${PYTHON_LIBRARY_NAME}" ${LIBRARY_NAME}
        ${PROJECT_NAME}-${PYTHON_LIBRARY_NAME}-wrap SOURCE_PYTHON_MODULE
        "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python-module-py.cc")
    elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python.hh")
      dynamic_graph_python_module(
        "${PY_NAME}/${PYTHON_LIBRARY_NAME}" ${LIBRARY_NAME}
        ${PROJECT_NAME}-${PYTHON_LIBRARY_NAME}-wrap MODULE_HEADER
        "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python.hh")
    endif()
  endif(BUILD_PYTHON_INTERFACE)
endforeach(plugin)

target_link_libraries(distribute-wrench PUBLIC eiquadprog::eiquadprog)

# main python module
set(${PROJECT_NAME}_PYTHON_FILES
    __init__.py
    main.py
    create_entities_utils.py
    meta_task_config.py
    meta_task_joint.py
    meta_task_pose.py
    motor_parameters.py)

# utils submodule
set(${PROJECT_NAME}_UTILS_PYTHON_FILES
    __init__.py
    control_utils.py
    plot_utils.py
    sot_utils.py
    filter_utils.py
    run_test_utils.py
    gazebo_utils.py)

# talos submodule
set(${PROJECT_NAME}_TALOS_PYTHON_FILES
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
    hip_flexibility_compensation_conf.py)

# simulation files
set(${PROJECT_NAME}_SIMULATION_FILES
    __init__.py
    test_dcm_zmp_control.py
    appli_dcm_zmp_control.py
    test_dcm_zmp_control_distribute.py
    appli_dcm_zmp_control_distribute.py
    test_dcm_zmp_control_ffdc.py
    appli_dcm_zmp_control_ffdc.py
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
    appli_hip_flexibility_compensation.py)

# simulation files
set(${PROJECT_NAME}_ROS_EXEC_FILES test_dcmZmpControl_file.py
                                   appli_dcmZmpControl_file.py)

if(BUILD_PYTHON_INTERFACE)
  foreach(python ${${PROJECT_NAME}_PYTHON_FILES})
    python_install_on_site(dynamic_graph/${PY_NAME} ${python})
  endforeach(python ${${PROJECT_NAME}_PYTHON_FILES})

  foreach(python ${${PROJECT_NAME}_TALOS_PYTHON_FILES})
    python_install_on_site(dynamic_graph/${PY_NAME}/talos ${python})
  endforeach(python ${${PROJECT_NAME}_TALOS_PYTHON_FILES})

  foreach(python ${${PROJECT_NAME}_UTILS_PYTHON_FILES})
    python_install_on_site(dynamic_graph/${PY_NAME}/utils ${python})
  endforeach(python ${${PROJECT_NAME}_UTILS_PYTHON_FILES})

  foreach(python ${${PROJECT_NAME}_SIMULATION_FILES})
    python_install_on_site(dynamic_graph/${PY_NAME}/test ${python})
  endforeach(python ${${PROJECT_NAME}_SIMULATION_FILES})

  foreach(python ${${PROJECT_NAME}_ROS_EXEC_FILES})
    install(PROGRAMS ${PY_NAME}/test/${python} DESTINATION lib/${PROJECT_NAME})
  endforeach(python ${${PROJECT_NAME}_ROS_EXEC_FILES})

endif(BUILD_PYTHON_INTERFACE)
