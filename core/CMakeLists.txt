# List of CPP (source) library files.
set(QpSolversEigen_SRC
  QpSolversEigen/Debug.cpp
  QpSolversEigen/NullSolver.cpp
  QpSolversEigen/Solver.cpp)

set(QpSolversEigen_HDR
  QpSolversEigen/Constants.hpp
  QpSolversEigen/Debug.hpp
  QpSolversEigen/NullSolver.hpp
  QpSolversEigen/Solver.hpp
  QpSolversEigen/SolverInterface.hpp
  QpSolversEigen/QpSolversEigen.hpp)

add_library(QpSolversEigen ${QpSolversEigen_SRC} ${QpSolversEigen_HDR})
# We want to keep the naming of library consistent, i.e.
# <prefix>qpsolvers-eigen<extension> for the core library and plugin loader
# <prefix>qpsolvers-eigen-osqp<extension> for the osqp plugin
# <prefix>qpsolvers-eigen-proxqp<extension> for the proxqp plugin and so on so forth
set_target_properties(QpSolversEigen PROPERTIES OUTPUT_NAME "qpsolvers-eigen")

target_include_directories(QpSolversEigen PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

target_link_libraries(QpSolversEigen PUBLIC Eigen3::Eigen PRIVATE sharedlibpp::sharedlibpp)
add_library(QpSolversEigen::QpSolversEigen ALIAS QpSolversEigen)

set_target_properties(QpSolversEigen PROPERTIES
  PUBLIC_HEADER "${QpSolversEigen_HDR}")

target_compile_features(QpSolversEigen PUBLIC cxx_std_17)

install(TARGETS QpSolversEigen
  EXPORT  ${PROJECT_NAME}
  PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/QpSolversEigen")
