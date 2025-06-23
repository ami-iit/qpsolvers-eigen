cmake_minimum_required(VERSION 3.16...3.31)

project(QpSolversEigen VERSION 0.2.0)

include(GNUInstallDirs)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

# Build shared libs on Windows
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

if(MSVC)
  set(CMAKE_DEBUG_POSTFIX "d")
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)

# Disable C and C++ compiler extensions.
# C/CXX_EXTENSIONS are ON by default to allow the compilers to use extended
# variants of the C/CXX language.
# However, this could expose cross-platform bugs in user code or in the headers
# of third-party dependencies and thus it is strongly suggested to turn
# extensions off.
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_EXTENSIONS OFF)


# Dependencies
find_package(Eigen3 REQUIRED)

set(QPSOLVERSEIGEN_USES_SYSTEM_SHAREDLIBPP_DEFAULT_VALUE OFF)
if(NOT DEFINED QPSOLVERSEIGEN_USES_SYSTEM_SHAREDLIBPP)
    find_package(sharedlibpp QUIET)
    set(QPSOLVERSEIGEN_USES_SYSTEM_SHAREDLIBPP_DEFAULT_VALUE ${sharedlibpp_FOUND})
endif()
option(QPSOLVERSEIGEN_USES_SYSTEM_SHAREDLIBPP "If ON, find sharedlibpp with find_package(sharedlibpp)" ${QPSOLVERSEIGEN_USES_SYSTEM_SHAREDLIBPP_DEFAULT_VALUE})
if(QPSOLVERSEIGEN_USES_SYSTEM_SHAREDLIBPP)
    find_package(sharedlibpp REQUIRED)
else()
    include(FetchContent)
    FetchContent_Declare(
        sharedlibpp
        URL https://github.com/ami-iit/sharedlibpp/archive/refs/tags/v0.0.3.zip
    )
    FetchContent_MakeAvailable(sharedlibpp)
endif()

set(QPSOLVERSEIGEN_USES_SYSTEM_YCM_DEFAULT_VALUE OFF)
if(NOT DEFINED QPSOLVERSEIGEN_USES_SYSTEM_YCM)
    find_package(YCM QUIET)
    set(QPSOLVERSEIGEN_USES_SYSTEM_YCM_DEFAULT_VALUE ${YCM_FOUND})
endif()
option(QPSOLVERSEIGEN_USES_SYSTEM_YCM "If ON, find ycm-cmake-modules with find_package(YCM)" ${QPSOLVERSEIGEN_USES_SYSTEM_YCM_DEFAULT_VALUE})
if(QPSOLVERSEIGEN_USES_SYSTEM_YCM)
    find_package(YCM REQUIRED)
else()
    include(FetchContent)
    FetchContent_Declare(
        YCM
        URL https://github.com/robotology/ycm-cmake-modules/archive/refs/tags/v0.16.9.zip
    )
    FetchContent_GetProperties(YCM)
    if(NOT YCM_POPULATED)
        FetchContent_Populate(YCM)
        include(${ycm_SOURCE_DIR}/tools/UseYCMFromSource.cmake)
    endif()
endif()

include(CMakePackageConfigHelpers)

option(QPSOLVERS_EIGEN_ENABLE_RPATH "Enable RPATH for this library" ON)
mark_as_advanced(QPSOLVERS_EIGEN_ENABLE_RPATH)
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
  LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
  DEPENDS QPSOLVERS_EIGEN_ENABLE_RPATH
  USE_LINK_PATH)

# Encourage user to specify a build type (e.g. Release, Debug, etc.), otherwise set it to Release.
if(NOT CMAKE_CONFIGURATION_TYPES)
  if(NOT CMAKE_BUILD_TYPE)
      message(STATUS "Setting build type to 'Release' as none was specified.")
      set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
  endif()
endif()

option(BUILD_TESTING "Create tests using CMake" OFF)
include(CTest)

# Set default build type to "Release" in single-config generators
if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING
        "Choose the type of build, recommended options are: Debug or Release" FORCE)
    endif()
    set(QPSOLVERSEIGEN_BUILD_TYPES "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${QPSOLVERSEIGEN_BUILD_TYPES})
endif()

# Add core library
add_subdirectory(core)

# Add plugins
add_subdirectory(plugins)

include(InstallBasicPackageFiles)
install_basic_package_files(${PROJECT_NAME}
  NAMESPACE QpSolversEigen::
  VERSION ${${PROJECT_NAME}_VERSION}
  COMPATIBILITY AnyNewerVersion
  VARS_PREFIX ${PROJECT_NAME}
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
  DEPENDENCIES Eigen3)

## Testing
include(AddQpSolversEigenUnitTest)
add_subdirectory(tests)
