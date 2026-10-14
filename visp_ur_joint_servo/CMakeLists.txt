cmake_minimum_required(VERSION 3.10)
project(visp_ur_joint_servo CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ViSP：只需要 core / visual_features / vs 三个模块即可完成关节空间控制律；
# detection / sensor / gui 仅用于示例跟踪器（RealSense + AprilTag），缺失时自动禁用。
find_package(VISP REQUIRED)
message(STATUS "ViSP ${VISP_VERSION} found")

# ur_rtde（可选）。未找到时只能 --sim 运行。
find_package(ur_rtde QUIET)
if(ur_rtde_FOUND)
  message(STATUS "ur_rtde ${ur_rtde_VERSION} found: real robot control enabled")
else()
  message(WARNING "ur_rtde not found: building simulation-only. Install from https://gitlab.com/sdurobotics/ur_rtde")
endif()

find_package(Threads REQUIRED)

add_executable(visp_ur_joint_servo src/visp_ur_joint_servo.cpp)
target_include_directories(visp_ur_joint_servo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${VISP_INCLUDE_DIRS})
target_link_libraries(visp_ur_joint_servo PRIVATE ${VISP_LIBRARIES} Threads::Threads)
target_compile_options(visp_ur_joint_servo PRIVATE ${VISP_DEFINITIONS})
if(ur_rtde_FOUND)
  target_compile_definitions(visp_ur_joint_servo PRIVATE HAVE_UR_RTDE)
  target_link_libraries(visp_ur_joint_servo PRIVATE ur_rtde::rtde)
endif()

add_executable(test_ur_kinematics src/test_ur_kinematics.cpp)
target_include_directories(test_ur_kinematics PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${VISP_INCLUDE_DIRS})
target_link_libraries(test_ur_kinematics PRIVATE ${VISP_LIBRARIES})
target_compile_options(test_ur_kinematics PRIVATE ${VISP_DEFINITIONS})

enable_testing()
add_test(NAME ur_kinematics COMMAND test_ur_kinematics)
add_test(NAME servo_simulation COMMAND visp_ur_joint_servo --sim --sim-duration 3 --rate 250 --stop-on-convergence)
