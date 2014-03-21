cmake_minimum_required(VERSION 2.8)
project( camcv_face )
SET(CMAKE_CXX_COMPILER g++)
SET(CMAKE_CC_COMPILER g++)
SET(COMPILE_DEFINITIONS -Werror)
SET(CMAKE_CXX_FLAGS -std=gnu++0x)
find_package( OpenCV REQUIRED )

include_directories(/opt/vc/userland/host_applications/linux/libs/bcm_host/include)

include_directories(/opt/vc/userland/interface/vcos)
include_directories(/opt/vc/userland)
include_directories(/opt/vc/userland/interface/vcos/pthreads)
include_directories(/opt/vc/userland/interface/vmcs_host/linux)
include_directories(/opt/vc/userland/interface/khronos/include)
include_directories(/opt/vc/userland/interface/khronos/common)
include_directories(/usr/include/c++/4.6.3)
include_directories(/usr/include/c++/4.6.3/arm-linux-gnueabihf)

include_directories(.)

add_executable(raspi_tracker_ros raspi_tracker_ros.cpp RaspiCamCV.cpp RaspiCamControl.c RaspiCLI.c read_settings.cpp)

target_link_libraries(raspi_tracker_ros /opt/vc/lib/libmmal_core.so /opt/vc/lib/libmmal_util.so /opt/vc/lib/libmmal_vc_client.so /opt/vc/lib/libvcos.so /opt/vc/lib/libbcm_host.so /opt/vc/lib/libGLESv2.so /opt/vc/lib/libEGL.so ${OpenCV_LIBS} )
