# Detect if we are doing a standalone build of this plugin, using an external qpsolvers-eigen
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  cmake_minimum_required(VERSION 3.16)
  project(qpsolvers-eigen-proxqp)
  find_package(QpSolversEigen REQUIRED)
  find_package(sharedlibpp REQUIRED)
  include(GNUInstallDirs)
  option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)
  option(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS "Export all symbols in Windows" ON)
endif()

find_package(proxsuite REQUIRED)

# The library name needs to be coherent with the scheme used in
# getShlibppLibraryNameFromSolverName, i.e. qpsolvers-eigen-<solverName>
add_library(qpsolvers-eigen-proxqp SHARED
    QpSolversEigenProxqp.cpp
)

target_compile_features(qpsolvers-eigen-proxqp PUBLIC cxx_std_17)

target_link_libraries(qpsolvers-eigen-proxqp
    PRIVATE
    QpSolversEigen::QpSolversEigen
    proxsuite::proxsuite
    sharedlibpp::sharedlibpp
)

install(TARGETS qpsolvers-eigen-proxqp
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
