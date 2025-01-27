cmake_minimum_required(VERSION 3.8)
project(omni_base_description)

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_python REQUIRED)

# PAL Robotics stricter build flags, other developers should not worry about this
find_package(ament_cmake_pal QUIET)

ament_auto_find_build_dependencies()

if(pal_module_cmake_FOUND)
  pal_register_modules(
    module/00_robot_state_publisher.yaml
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

  foreach(laser_model no-laser hokuyo sick-551 sick-561 sick-571)
      foreach(add_on_module no-add-on)
      add_launch_test(
        test/test_description.launch.py
        TARGET "omni_base_description_${laser_model}_${add_on_module}"
        ARGS "laser_model:=${laser_model}" "add_on_module:=${add_on_module}"
      )
      endforeach()
  endforeach()
endif()

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_auto_package(INSTALL_TO_SHARE rviz launch meshes config robots ros2_control urdf)
