cmake_minimum_required(VERSION 3.8)
project(mu_bringup)

# C++ 표준/경고설정
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES)
endif()

# ===== SLLIDAR SDK 설정 =====
set(SLLIDAR_SDK_PATH "./sdk/")

file(GLOB SLLIDAR_SDK_SRC 
  "${SLLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/dataunpacker/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/dataunpacker/unpacker/*.cpp"
)

# ===== 의존성 =====
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
# rclpy는 런타임 의존이면 CMake에서 REQUIRED 안 해도 되고, package.xml에 exec_depend로만 넣어도 됩니다.

# ===== 파이썬 패키지 설치 =====
ament_python_install_package(${PROJECT_NAME})

# 파이썬 실행 스크립트 설치
install(PROGRAMS
  ${PROJECT_NAME}/scan_crop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME scan_crop
)

# ===== Include 경로 =====
include_directories(
  ${SLLIDAR_SDK_PATH}/include
  ${SLLIDAR_SDK_PATH}/src
)

# ===== C++ 실행파일 (LiDAR 노드) =====
add_executable(lidar_node src/lidar_node.cpp ${SLLIDAR_SDK_SRC})
ament_target_dependencies(lidar_node rclcpp sensor_msgs std_srvs)
install(TARGETS lidar_node DESTINATION lib/${PROJECT_NAME})

# ===== 런치/설정 파일 =====
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# ===== 테스트/내보내기 =====
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rclcpp sensor_msgs std_srvs)
ament_package()
