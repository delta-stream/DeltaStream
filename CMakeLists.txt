cmake_minimum_required(VERSION 3.29)
project(delta_stream)

set(CMAKE_CXX_STANDARD 20)


set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
list(APPEND CMAKE_PREFIX_PATH "libs/open3d/lib/cmake")

# Set to OFF to build static libraries
#set(BUILD_SHARED_LIBS OFF)

find_package(draco CONFIG REQUIRED)
find_package(boost_asio CONFIG REQUIRED)
find_package(boost_lockfree  CONFIG REQUIRED)
find_package(OpenCV REQUIRED)
find_package(FFMPEG REQUIRED)


include_directories( ${OpenCV_INCLUDE_DIRS} )
# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# Make project require C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

find_library(REALSENSE2_FOUND realsense2 HINTS ${LIBRARY_DIR} REQUIRED)
if(NOT REALSENSE2_FOUND)
    SET(REALSENSE2_FOUND "realsense2")
    message(WARN "Failed to find_library(realsense2)")
endif()
include_directories(server ./third-party/imgui ${ADDITIONAL_INCLUDE_DIRS})


# Open3D

# Find installed Open3D, this imports
#   Open3D_INCLUDE_DIRS
#   Open3D_LIBRARIES
#   Open3D_LIBRARY_DIRS
#   Open3D_C_FLAGS
#   Open3D_CXX_FLAGS
#   Open3D_EXE_LINKER_FLAGS
if(WIN32)
    find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/CMake)
else()
    find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/lib/CMake)
    list(APPEND Open3D_LIBRARIES dl)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${Open3D_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Open3D_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${Open3D_EXE_LINKER_FLAGS}")

if(WIN32)
elseif(CYGWIN)
elseif(APPLE)
elseif(UNIX)
    add_definitions(-DUNIX)
    add_compile_options(-Wno-deprecated-declarations)
    add_compile_options(-Wno-unused-result)
    add_definitions(-O3)
endif(WIN32)


# link_directories must be before add_executable
link_directories(${Open3D_LIBRARY_DIRS})


add_executable(server main.cpp
        realsense.h
        draco.h
        logging.h
        tools.h
        FrameData.h
        FrameDataType.h
        EncodedFrame.h
        FrameMapManager.cpp
        FrameMapManager.h
        EncodedFrameDTO.h
        DecodedFrameDTO.h
        frame_preppers/BlockUpdatePrepper.cpp
        frame_preppers/BlockUpdatePrepper.h
        frame_preppers/MotionVectorPrepper.cpp
        frame_preppers/MotionVectorPrepper.h
        DeltaMotion.h
        config.h
        FfmpegToolbox.cpp
        FfmpegToolbox.h
        AVBufferData.h
        Benchmark.h
        raw_frame_producers/RealSenseRawFrameProducer.cpp
        raw_frame_producers/RealSenseRawFrameProducer.h
        custom_types/DepthConfig.h
        custom_types/RawFrame.h
        raw_frame_producers/ImageRawFrameProducer.cpp
        raw_frame_producers/ImageRawFrameProducer.h
        custom_types/MultiCameraFrameData.h
        custom_types/MultiCameraEncodedFrameDTO.h
        custom_types/MultiCameraDecodedFrameDTO.h
        custom_types/MVFramePrepData.h
        ServerBenchmarkMap.h
        MVFrameLatencyReport.cpp
        MVFrameLatencyReport.h
        FPSCounter.h
        custom_types/ClientFeedback.h
        MovingAverage.hpp
)
add_executable(client client.cpp
        realsense.h
        draco.h
        logging.h
        tools.h
        FrameData.h
        FrameDataType.h
        EncodedFrame.h
        FrameMapManager.cpp
        FrameMapManager.h
        EncodedFrameDTO.h
        DecodedFrameDTO.h
        DeltaMotion.h
        config.h
        encoded_frame_handlers/PartialFrameHandler.cpp
        encoded_frame_handlers/PartialFrameHandler.h
        Benchmark.h
        custom_types/DepthConfig.h
        custom_types/MultiCameraEncodedFrameDTO.h
        custom_types/MultiCameraDecodedFrameDTO.h
        DecodedFrameDTOManager.cpp
        DecodedFrameDTOManager.h
        ClientBenchmarkMap.h
        MVFrameLatencyReport.cpp
        MVFrameLatencyReport.h
        FPSCounter.h
        custom_types/ClientFeedback.h
        MovingAverage.hpp
)

# Create an interface library for Cereal
add_library(cereal INTERFACE)

target_include_directories(cereal INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/libs)

target_link_directories(server PRIVATE ${FFMPEG_LIBRARY_DIRS})
target_link_libraries(server
        PRIVATE draco::draco ${Open3D_LIBRARIES} ${REALSENSE2_FOUND} cereal
        Boost::asio Boost::lockfree  ${OpenCV_LIBS} ${FFMPEG_LIBRARIES} ${DEPENDENCIES}
)
target_include_directories(server PUBLIC ${Open3D_INCLUDE_DIRS}  ${Boost_INCLUDE_DIRS} ${FFMPEG_INCLUDE_DIRS})

target_link_libraries(client
        PRIVATE draco::draco ${Open3D_LIBRARIES} ${REALSENSE2_FOUND} cereal
        Boost::asio Boost::lockfree  ${OpenCV_LIBS}  ${DEPENDENCIES}
)
target_include_directories(client PUBLIC ${Open3D_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )

install(
        TARGETS

        server client

        RUNTIME DESTINATION
        ${CMAKE_INSTALL_PREFIX}/bin
)