cmake_minimum_required(VERSION 3.12)
project(asteroid_mining_conversions)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(asteroid_mining REQUIRED)
find_package(asteroid_mining_interfaces REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} src/conversions.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC
    ${asteroid_mining_interfaces_TARGETS}
    asteroid_mining::asteroid_mining_carriers
)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

ament_export_dependencies(asteroid_mining asteroid_mining_interfaces)
ament_package()
