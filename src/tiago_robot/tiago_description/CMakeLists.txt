cmake_minimum_required(VERSION 3.8)
project(tiago_description)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(pal_module_cmake_FOUND)
  pal_register_modules(
    module/10_robot_state_publisher.yaml
  )
elseif(DEFINED ENV{PAL_DISTRO})
  message(FATAL_ERROR "The pal_module_cmake package was not found. Modules not registered.")
endif()

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  #  find_package(rostest REQUIRED)
    foreach(arm_type tiago-arm no-arm)
      foreach(base_type pmb2 omni_base)
        add_launch_test(
          test/test_description.launch.py
          TARGET "tiago_description_${base_type}_${arm_type}"
          ARGS "arm_type:=${arm_type}" "base_type:=${base_type}"
        )
      endforeach()

        # Separated to speed up tests, otherwise it's over 13min
        foreach(end_effector no-end-effector pal-gripper pal-hey5 custom)
            foreach(ft_sensor no-ft-sensor schunk-ft)
              add_launch_test(
                test/test_description.launch.py
                TARGET "tiago_description_${arm_type}_${end_effector}_${ft_sensor}"
                ARGS "arm_type:=${arm_type}" "end_effector:=${end_effector}" "ft_sensor:=${ft_sensor}"
              )
            endforeach()
        endforeach()

        foreach(laser_model no-laser hokuyo sick-551 sick-561 sick-571)
          add_launch_test(
            test/test_description.launch.py
            TARGET "tiago_description_${arm_type}_${laser_model}"
            ARGS "arm_type:=${arm_type}" "laser_model:=${laser_model}"
          )
        endforeach()
        foreach(camera_model no-camera orbbec-astra orbbec-astra-pro asus-xtion)
          add_launch_test(
            test/test_description.launch.py
            TARGET "tiago_description_${arm_type}_${camera_model}"
            ARGS "arm_type:=${arm_type}" "camera_model:=${camera_model}"
          )
        endforeach()
    endforeach()
endif()

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_auto_package(INSTALL_TO_SHARE launch meshes robots ros2_control urdf config)
