PROJECT(telemetry_telecommand)
cmake_minimum_required(VERSION 2.6)

add_definitions(-Wall)
add_definitions(-DBASE_LOG_NAMESPACE=${PROJECT_NAME})
add_definitions(-DBASE_LOG_INFO)

SET (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/.orogen/config")
INCLUDE(telemetry_telecommandBase)
