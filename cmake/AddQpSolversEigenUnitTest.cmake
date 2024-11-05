# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

include(CMakeDependentOption)

cmake_dependent_option(QPSOLVERSEIGEN_RUN_Valgrind_tests
  "Run Valgrind tests?" OFF
  "VALGRIND_FOUND" OFF)

if (QPSOLVERSEIGEN_RUN_Valgrind_tests)
    set(CTEST_MEMORYCHECK_COMMAND ${VALGRIND_PROGRAM})
    set(MEMORYCHECK_COMMAND ${VALGRIND_PROGRAM})
    if (APPLE)
        set(MEMORYCHECK_SUPPRESSIONS "--suppressions=${PROJECT_SOURCE_DIR}/cmake/valgrind-macos.supp")
    else ()
        set(MEMORYCHECK_SUPPRESSIONS "")
    endif ()
    set(MEMORYCHECK_COMMAND_OPTIONS "--leak-check=full --error-exitcode=1 ${MEMORYCHECK_SUPPRESSIONS}"  CACHE STRING "Options to pass to the memory checker")
    mark_as_advanced(MEMORYCHECK_COMMAND_OPTIONS)
    set(MEMCHECK_COMMAND_COMPLETE "${MEMORYCHECK_COMMAND} ${MEMORYCHECK_COMMAND_OPTIONS}")
    separate_arguments(MEMCHECK_COMMAND_COMPLETE)
endif()

if (BUILD_TESTING)
    find_package(Catch2 REQUIRED)
endif()


function(add_qpsolverseigen_test)

    if(BUILD_TESTING)

      set(options)
      set(oneValueArgs NAME)
      set(multiValueArgs SOURCES LINKS COMPILE_DEFINITIONS)

      set(prefix "qpsolvers_eigen")

      cmake_parse_arguments(${prefix}
          "${options}"
          "${oneValueArgs}"
          "${multiValueArgs}"
          ${ARGN})

      set(name ${${prefix}_NAME})
      set(unit_test_files ${${prefix}_SOURCES})

      set(targetname ${name}UnitTests)
      add_executable(${targetname}
          "${unit_test_files}")

      target_link_libraries(${targetname} PRIVATE Catch2::Catch2WithMain ${${prefix}_LINKS})
      target_compile_definitions(${targetname} PRIVATE CATCH_CONFIG_FAST_COMPILE CATCH_CONFIG_DISABLE_MATCHERS)
      target_compile_features(${targetname} PUBLIC cxx_std_17)

      add_test(NAME ${targetname} COMMAND ${targetname})
      target_compile_definitions(${targetname} PRIVATE ${${prefix}_COMPILE_DEFINITIONS})

      # Append the value to the SHLIBPP_PLUGIN_PATH environment variable
      if(WIN32)
        set_property(TEST ${targetname} APPEND PROPERTY
            ENVIRONMENT_MODIFICATION "SHLIBPP_PLUGIN_PATH=path_list_prepend:${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
      else()
        set_property(TEST ${targetname} APPEND PROPERTY
            ENVIRONMENT_MODIFICATION "SHLIBPP_PLUGIN_PATH=path_list_prepend:${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
      endif()

      if(QPSOLVERSEIGEN_RUN_Valgrind_tests)
        add_test(NAME memcheck_${targetname} COMMAND ${MEMCHECK_COMMAND_COMPLETE} $<TARGET_FILE:${targetname}>)
      endif()

    endif()

endfunction()
