cmake_minimum_required(VERSION 3.15)
project(booster_lowlevel LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# pybind11 from the Python that builds the module (`python -m pybind11 --cmakedir`).
find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
execute_process(
  COMMAND "${Python_EXECUTABLE}" -m pybind11 --cmakedir
  OUTPUT_VARIABLE pybind11_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE)
find_package(pybind11 CONFIG REQUIRED)
find_package(Threads REQUIRED)

# The SDK's prebuilt static library for this machine (x86_64 or aarch64).
set(SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(SDK_ARCH "${CMAKE_SYSTEM_PROCESSOR}")
if(SDK_ARCH MATCHES "^(AMD64|amd64)$")
  set(SDK_ARCH x86_64)
endif()
set(SDK_LIBRARY "${SDK_ROOT}/lib/${SDK_ARCH}/libbooster_robotics_sdk.a")
if(NOT EXISTS "${SDK_LIBRARY}")
  message(FATAL_ERROR "Booster Robotics SDK library not found: ${SDK_LIBRARY}")
endif()

pybind11_add_module(booster_lowlevel booster_lowlevel.cpp)
target_include_directories(booster_lowlevel PRIVATE "${SDK_ROOT}/include")
target_link_libraries(booster_lowlevel PRIVATE "${SDK_LIBRARY}" Threads::Threads ${CMAKE_DL_LIBS} rt)
