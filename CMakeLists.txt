cmake_minimum_required(VERSION 3.15)


project(Simpl_Drone_Simulator CXX)

find_package(CppUTest CONFIG REQUIRED)

set(CMAKE_CXX_STANDARD 20)

# add_executable(myEXE helloworld.cpp ./HELLOWORLD/helper.cpp )
add_executable(run_tests ./tests/tests_main.cpp ./tests/dynamic_test_group.cpp ./src/dynamics.cpp ./tests/controller_test_group.cpp ./src/controller.cpp)


# target_link_libraries(myEXE 
# PRIVATE
#   glfw
#   implot::implot
#   imgui::imgui
# )

target_link_libraries(run_tests
  PRIVATE
    CppUTest
    CppUTestExt)