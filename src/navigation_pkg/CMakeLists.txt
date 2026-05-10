cmake_minimum_required(VERSION 3.8)
project(navigation_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY 
  urdf
  worlds
  launch
  rviz
  config
  maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  navigation_pkg/webcam_publisher.py 
  DESTINATION lib/${PROJECT_NAME}
  RENAME webcam_publisher
)

install(PROGRAMS
  navigation_pkg/person_detector.py 
  DESTINATION lib/${PROJECT_NAME}
  RENAME person_detector
)

install(PROGRAMS
  navigation_pkg/battery_simulator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME battery_simulator
)

install(PROGRAMS
  navigation_pkg/auto_dock_manager.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME auto_dock_manager
)

install(PROGRAMS
  navigation_pkg/dock_pose_broadcaster.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dock_pose_broadcaster
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
