cmake_minimum_required(VERSION 3.5)
project(position_control)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

install(PROGRAMS
  nodes/kalman_filter.py
  nodes/ranges_debugger.py
  nodes/yaw_controller.py
  nodes/grasp.py
  nodes/object_detection.py
  nodes/image_camera_synchronizer.py
  nodes/pos_controller.py
  nodes/pos_setpoint_pub.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
