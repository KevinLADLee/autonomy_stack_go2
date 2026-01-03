cmake_minimum_required(VERSION 3.5)
project(tare_tsp_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SolveTSP.srv"
  DEPENDENCIES
)

#############
## Install ##
#############

# Install Python scripts
install(PROGRAMS
  scripts/tsp_solver_service.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies for service interfaces
ament_export_dependencies(rosidl_default_runtime)

ament_package()



