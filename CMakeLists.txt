cmake_minimum_required(VERSION 2.8)
project(zebrafish_tracker_frontend)

set( 64BIT "x86_64")

# For preprocessor flags
add_definitions( -D_COPY_IMAGE_BUFFER_ )

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

add_executable(start_frontend src/main.cpp)
target_link_libraries(start_frontend ${UEYE_LIBRARY} ${OpenCV_LIBS} ${Boost_LIBRARIES})