cmake_minimum_required(VERSION 3.8)
project(monodepth_navigation)

set(DEPENDENCIES
  ament_cmake
  ament_cmake_python
  rclpy
  sensor_msgs
  std_msgs
  cv_bridge
  tf2_ros
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

install(
  DIRECTORY launch resource rviz tmux
  DESTINATION share/${PROJECT_NAME}
)


install(
  PROGRAMS
    create_python_env.sh
    install_requirements.sh
  DESTINATION share/${PROJECT_NAME}
)


file(GLOB PYTHON_ENV_FILES python-env/*)
install(FILES ${PYTHON_ENV_FILES} DESTINATION share/${PROJECT_NAME}/python-env)


ament_python_install_package(monodepth PACKAGE_DIR src/monodepth)


install(
  PROGRAMS
    src/monodepth/monocular_depth_estimator.py
    src/monodepth/navigation_controller.py
    src/monodepth/image_undistorter.py
    src/monodepth/camera_processor.py
    src/pointcloud_processor.py
    src/scale_estimator.py
    src/midas_extension.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()