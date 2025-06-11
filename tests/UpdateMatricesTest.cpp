/**
 * @file UpdateMatricesTest.cpp
 * @author Giulio Romualdi
 * @copyright Released under the terms of the BSD 3-Clause License
 * @date 2020
 */

// Include macros common to all test
#include "QpSolversEigenCommonTestMacros.hpp"

// Catch2
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators_all.hpp>

// QpSolversEigen
#include <QpSolversEigen/QpSolversEigen.hpp>

#include <iostream>

// colors
#define ANSI_TXT_GRN "\033[0;32m"
#define ANSI_TXT_MGT "\033[0;35m" // Magenta
#define ANSI_TXT_DFT "\033[0;0m" // Console default
#define GTEST_BOX "[     cout ] "
#define COUT_GTEST ANSI_TXT_GRN << GTEST_BOX // You could add the Default
#define COUT_GTEST_MGT COUT_GTEST << ANSI_TXT_MGT


TEST_CASE("QPProblem - UpdateMatricesTest")
{
    Eigen::Matrix<double, 2, 2> H;
    Eigen::SparseMatrix<double> H_s;
    Eigen::Matrix<double, 3, 2> A;
    Eigen::SparseMatrix<double> A_s;
    Eigen::Matrix<double, 3, 2> C;
    Eigen::SparseMatrix<double> C_s;
    Eigen::Matrix<double, 2, 1> gradient;
    Eigen::Matrix<double, 3, 1> lowerBound;
    Eigen::Matrix<double, 3, 1> upperBound;
    Eigen::Matrix<double, 3, 1> equalityConstraintVector;

    std::string solverName = QPSOLVERSEIGEN_SOLVERS_TO_TEST;
    QpSolversEigen::Solver solver;
    REQUIRE(solver.instantiateSolver(solverName));

    // hessian matrix
    H << 4, 0, 0, 2;
    H_s = H.sparseView();
    H_s.pruned(0.01);

    // inequality constraint matrix
    A << 1, 1, 1, 0, 0, 1;
    A_s = A.sparseView();

    // equality constraint matrix
    C << 1, 1, 1, 0.5, 0, 1;
    C_s = C.sparseView();

    gradient << 1, 1;
    lowerBound << 1, 0, 0;
    upperBound << 1, 0.7, 0.7;
    equalityConstraintVector << 1, 0.5, 0;

    // Set osqp-specific parameters
    if (solver.getSolverName() == "osqp")
    {
        solver.setBooleanParameter("verbose", false);
        solver.setIntegerParameter("scaling", 0);
    }
    solver.setNumberOfVariables(2);
    solver.setNumberOfInequalityConstraints(3);
    solver.setNumberOfEqualityConstraints(3);
    REQUIRE(solver.setHessianMatrix(H_s));
    REQUIRE(solver.setGradient(gradient));
    REQUIRE(solver.setInequalityConstraintsMatrix(A_s));
    REQUIRE(solver.setLowerBound(lowerBound));
    REQUIRE(solver.setUpperBound(upperBound));
    REQUIRE(solver.setEqualityConstraintsMatrix(C_s));
    REQUIRE(solver.setEqualityConstraintsVector(equalityConstraintVector));

    REQUIRE(solver.initSolver());
    REQUIRE(solver.solveProblem() == QpSolversEigen::ErrorExitFlag::NoError);

    auto solution = solver.getSolution();
    std::cout << COUT_GTEST_MGT << "Solution [" << solution(0) << " " << solution(1) << "]"
              << ANSI_TXT_DFT << std::endl;

    // update hessian matrix
    H << 4, 0, 0, 2;
    H_s = H.sparseView();
    A << 2, 1, 1, 0, 0, 1;
    A_s = A.sparseView();
    C << 2, 0, 2, 0.5, 0, 1;
    C_s = C.sparseView();

    REQUIRE(solver.updateHessianMatrix(H_s));
    REQUIRE(solver.updateInequalityConstraintsMatrix(A_s));
    REQUIRE(solver.updateEqualityConstraintsMatrix(C_s));
    REQUIRE(solver.solveProblem() == QpSolversEigen::ErrorExitFlag::NoError);

    solution = solver.getSolution();
    std::cout << COUT_GTEST_MGT << "Solution [" << solution(0) << " " << solution(1) << "]"
              << ANSI_TXT_DFT << std::endl;

    // update hessian matrix
    H << 1, 1, 1, 2;
    H_s = H.sparseView();
    A << 1, 1, 1, 0.4, 0, 1;
    A_s = A.sparseView();
    C << 1, 1, 0.1, 0.5, 0, 0.1;

    REQUIRE(solver.updateHessianMatrix(H_s));
    REQUIRE(solver.updateInequalityConstraintsMatrix(A_s));
    REQUIRE(solver.updateEqualityConstraintsMatrix(C_s));
    REQUIRE(solver.solveProblem() == QpSolversEigen::ErrorExitFlag::NoError);

    solution = solver.getSolution();
    std::cout << COUT_GTEST_MGT << "Solution [" << solution(0) << " " << solution(1) << "]"
              << ANSI_TXT_DFT << std::endl;
};
