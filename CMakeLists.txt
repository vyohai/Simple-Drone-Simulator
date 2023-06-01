cmake_minimum_required(VERSION 3.15)


project(Simple_Drone_Simulator CXX)

find_package(CppUTest CONFIG REQUIRED)

find_package(Armadillo REQUIRED)
find_package(sciplot CONFIG REQUIRED)
         




set(CMAKE_CXX_STANDARD 20)

add_executable(App 
              ./src/mainApp.cpp 
              ./src/dynamics/forcesAndMoments/forcesAndMoments.cpp 
              ./src/dynamics/framesConversion/framesConversion.cpp
              ./src/updateEquations/updateEquations.cpp
              ./src/controller/controller.cpp
              ./src/visualization/visualization.cpp
              ./src/interface/interface.cpp)

add_executable(run_tests 
              ./tests/tests_main.cpp 
              ./tests/dynamic_test_group.cpp 
              ./tests/updateEquations_test_group.cpp
              ./tests/controller_test_group.cpp 
              ./src/dynamics/forcesAndMoments/forcesAndMoments.cpp 
              ./src/dynamics/framesConversion/framesConversion.cpp 
              ./src/updateEquations/updateEquations.cpp
              ./src/controller/controller.cpp
              ./src/visualization/visualization.cpp)


target_include_directories(App PRIVATE ${ARMADILLO_INCLUDE_DIRS})
target_include_directories(run_tests PRIVATE ${ARMADILLO_INCLUDE_DIRS})

target_link_libraries(run_tests
  PRIVATE
    sciplot::sciplot  
    CppUTest
    CppUTestExt
    ${ARMADILLO_LIBRARIES})

    target_link_libraries(App
    PRIVATE
      sciplot::sciplot 
      ${ARMADILLO_LIBRARIES})