cmake_minimum_required(VERSION 3.16...3.31)

project(QpSolversEigen-MPCExample)

find_package(QpSolversEigen REQUIRED)

add_executable(QpSolversEigen-MPCExaple MPCExample.cpp)
target_link_libraries(QpSolversEigen-MPCExaple QpSolversEigen::QpSolversEigen)
target_compile_features(QpSolversEigen-MPCExaple PUBLIC cxx_std_20)
