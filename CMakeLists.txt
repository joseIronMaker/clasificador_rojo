cmake_minimum_required(VERSION 3.8)
project(clasificador_rojo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(control_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(webots_ros2_driver REQUIRED)
find_package(pluginlib REQUIRED)

# ---------- deteccion_rojo (nodo C++ + OpenCV) ----------
add_executable(deteccion_rojo src/deteccion_rojo.cpp)
ament_target_dependencies(deteccion_rojo
  rclcpp sensor_msgs std_msgs geometry_msgs cv_bridge)
target_link_libraries(deteccion_rojo ${OpenCV_LIBS})

# ---------- orquestador (nodo C++: máquina de estados) ----------
add_executable(orquestador src/orquestador.cpp)
ament_target_dependencies(orquestador
  rclcpp rclcpp_action std_msgs std_srvs control_msgs nav2_msgs geometry_msgs)

# ---------- controlador_banda (plugin C++ de webots_ros2_driver) ----------
add_library(controlador_banda SHARED src/controlador_banda.cpp)
ament_target_dependencies(controlador_banda
  rclcpp std_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs webots_ros2_driver pluginlib)
# Exporta el plugin para que el driver lo cargue vía pluginlib
pluginlib_export_plugin_description_file(webots_ros2_driver plugins/controlador_banda.xml)

install(TARGETS deteccion_rojo orquestador
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS controlador_banda
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# ---------- brazo (Etapa 2: MoveIt2 C++). Se compila cuando exista src/brazo.cpp ----------
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/brazo.cpp")
  find_package(moveit_ros_planning_interface REQUIRED)
  find_package(moveit_msgs REQUIRED)
  find_package(shape_msgs REQUIRED)
  find_package(trajectory_msgs REQUIRED)
  add_executable(brazo src/brazo.cpp)
  ament_target_dependencies(brazo
    rclcpp rclcpp_action std_msgs std_srvs geometry_msgs control_msgs trajectory_msgs
    tf2 moveit_msgs shape_msgs moveit_ros_planning_interface)
  install(TARGETS brazo DESTINATION lib/${PROJECT_NAME})
endif()

# ---------- Scripts Python (PLC/monitor + panel de control) ----------
install(PROGRAMS scripts/plc_sim.py scripts/control_gui.py scripts/monitor_condicion.py
  DESTINATION lib/${PROJECT_NAME})

# ---------- Recursos (mundos, urdf, configs, lanzamientos) ----------
install(DIRECTORY worlds urdf config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
