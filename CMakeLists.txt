CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12.1)

set(PROG_NAME "depth2rgb")
project(${PROG_NAME} CXX)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
IF( EXISTS "${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json" )
  EXECUTE_PROCESS( COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json
    ${CMAKE_CURRENT_SOURCE_DIR}/compile_commands.json
  )
ENDIF()

# library dependencies
find_package(OpenCV REQUIRED)

find_library(Freenect2_LIBS freenect2)
if(NOT Freenect2_LIBS)
  message(FATAL_ERROR "libfreenect2 library not found.")
endif()

# test exe configuration
set(SOURCES "src/main.cpp")
set(LIBRARIES ${Freenect2_LIBS} ${OpenCV_LIBS})
# include_directories(BEFORE "include")
add_executable(${PROG_NAME} ${SOURCES})
target_link_libraries(${PROG_NAME} ${LIBRARIES})
