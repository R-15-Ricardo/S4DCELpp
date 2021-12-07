cmake_minimum_required(VERSION 3.20)
project(S4DCELpp)

set(CMAKE_CXX_STANDARD 11)

add_subdirectory(vendor/pybind11)

pybind11_add_module(PyS4DCEL src/module_main.cpp)
