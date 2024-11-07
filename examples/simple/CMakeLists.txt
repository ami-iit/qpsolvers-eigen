cmake_minimum_required(VERSION 3.16...3.31)

project(QpSolversEigen-SimpleExample)

find_package(QpSolversEigen REQUIRED)

add_executable(QpSolversEigen-SimpleExample SimpleExample.cpp)
target_link_libraries(QpSolversEigen-SimpleExample QpSolversEigen::QpSolversEigen)
