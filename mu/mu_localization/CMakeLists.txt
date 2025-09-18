cmake_minimum_required(VERSION 3.8)
project(mu_localization)

find_package(ament_cmake REQUIRED)

# install launch & config
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
