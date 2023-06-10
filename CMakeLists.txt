cmake_minimum_required(VERSION 3.15)


project(Simple_Drone_Simulator CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(CppUTest CONFIG REQUIRED)

find_package(Armadillo REQUIRED)

find_package(matplot++ CONFIG REQUIRED)


add_executable(App 
              ./src/mainApp.cpp 
              ./src/dynamics/forcesAndMoments/forcesAndMoments.cpp 
              ./src/dynamics/framesConversion/framesConversion.cpp
              ./src/updateEquations/updateEquations.cpp
              ./src/controller/controller.cpp
              ./src/visualization/visualization.cpp
              ./src/interface/interface.cpp
              ./src/controllerTuner/controllerTuner.cpp
              ./src/sensors/sensors.cpp)

add_executable(run_tests 
              ./tests/tests_main.cpp 
              ./tests/dynamic_test_group.cpp 
              ./tests/updateEquations_test_group.cpp
              ./tests/controller_test_group.cpp 
              ./tests/controllerTuner_test_group.cpp
              ./src/dynamics/forcesAndMoments/forcesAndMoments.cpp 
              ./src/dynamics/framesConversion/framesConversion.cpp 
              ./src/updateEquations/updateEquations.cpp
              ./src/controller/controller.cpp
              ./src/controllerTuner/controllerTuner.cpp
              ./src/visualization/visualization.cpp
              ./src/sensors/sensors.cpp
              ./src/interface/interface.cpp)


target_include_directories(App PRIVATE ${ARMADILLO_INCLUDE_DIRS})
target_include_directories(run_tests PRIVATE ${ARMADILLO_INCLUDE_DIRS})

target_link_libraries(run_tests
  PRIVATE
    Matplot++::cimg
    std::filesystem
    Matplot++::matplot
    Matplot++::nodesoup
    CppUTest
    CppUTestExt
    ${ARMADILLO_LIBRARIES})

target_link_libraries(App
  PRIVATE
  Matplot++::cimg
  std::filesystem
  Matplot++::matplot
  Matplot++::nodesoup
  ${ARMADILLO_LIBRARIES})