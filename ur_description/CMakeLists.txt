cmake_minimum_required(VERSION 3.5)
project(ur_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

install(
  DIRECTORY config launch meshes rviz urdf test
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(ur_urdf_xacro test/test_ur_urdf_xacro.py)
  ament_add_pytest_test(view_ur_launch test/test_view_ur_launch.py)
endif()
 
install(PROGRAMS
  scripts/task1a.py
  scripts/task1b.py
  scripts/task2a.py
  scripts/armed/spawner_box_com.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY scripts/armed/pyarmor_runtime_000000
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
