cmake_minimum_required(VERSION 2.8)
project(zebrafish_tracker_frontend)

# For debugging with GDB
set(CMAKE_BUILD_TYPE Debug)
# For C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set( 64BIT "x86_64")

# For preprocessor flags
#add_definitions( -D_GREYSCALE_IMAGE_ )
#add_definitions( -D_VERBOSE_MODE_ )

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

#Check for proper architecture
EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )

if( NOT UNIX )
  message( FATAL_ERROR "Currently, only Unix systems are supported!")
endif()

#message( STATUS "Architecture: ${ARCHITECTURE}" )
if( ${ARCHITECTURE} STREQUAL 64BIT )
  set( UEYE_LIBRARY "/usr/lib/libueye_api64.so.4.72" )

  set( SERIAL_LIBRARY "/usr/lib/libserial.so" )
  find_package(Threads)
  message( STATUS "x86_64 architecture detected.")
else()
  message( FATAL_ERROR "Architecture other than ${64BIT} currently not supported." )
endif()

#link_directories(/usr/

include_directories(
  include
  /usr/include
  ${OpenCV_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  )

link_directories(${Boost_LIBRARY_DIRS})


#add_library(${UEYECAM_LIB_NAME} ${UEYECAM_LIB_SOURCES} ${UEYECAM_LIB_HEADERS})
#target_link_libraries(${UEYECAM_LIB_NAME} ${UEYE_LIBRARY})

#add_library(${UEYECAM_NODELET_NAME} ${UEYECAM_NODELET_SOURCES} ${UEYECAM_NODELET_HEADERS})
#target_link_libraries(${UEYECAM_NODELET_NAME} ${catkin_LIBRARIES} ${UEYECAM_LIB_NAME})

#Make a library from the classes
add_library(camera_interface src/camera_interface.cpp)
add_library(image_processing src/image_processing.cpp)
add_library(xyPlotter src/xyPlotter.cpp)

add_executable(start_frontend src/main.cpp)
target_link_libraries(start_frontend camera_interface image_processing xyPlotter ${SERIAL_LIBRARY} ${UEYE_LIBRARY} ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_executable(uart_test src/uart_test.cpp)
target_link_libraries(uart_test ${SERIAL_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})

add_executable(calibrate src/calibrate.cpp)
target_link_libraries(calibrate camera_interface xyPlotter ${UEYE_LIBRARY} ${SERIAL_LIBRARY} ${OpenCV_LIBS} ${Boost_LIBRARIES})