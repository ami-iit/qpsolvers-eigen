
add_qpsolverseigen_test(
  NAME QP
  SOURCES QPTest.cpp
  LINKS QpSolversEigen::QpSolversEigen)

add_qpsolverseigen_test(
  NAME UpdateMatrices
  SOURCES UpdateMatricesTest.cpp
  LINKS QpSolversEigen::QpSolversEigen)

add_qpsolverseigen_test(
  NAME MPC
  SOURCES MPCTest.cpp
  LINKS QpSolversEigen::QpSolversEigen
  COMPILE_DEFINITIONS _USE_MATH_DEFINES)

add_qpsolverseigen_test(
  NAME MPCUpdateMatrices
  SOURCES MPCUpdateMatricesTest.cpp
  LINKS QpSolversEigen::QpSolversEigen
  COMPILE_DEFINITIONS _USE_MATH_DEFINES)

  add_qpsolverseigen_test(
    NAME MPCEqualityConstraints
    SOURCES MPCEqualityConstraintsTest.cpp
    LINKS QpSolversEigen::QpSolversEigen
    COMPILE_DEFINITIONS _USE_MATH_DEFINES)

