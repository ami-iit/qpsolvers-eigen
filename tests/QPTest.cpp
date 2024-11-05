/**
 * @file QPTest.cpp
 * @author Giulio Romualdi
 * @copyright Released under the terms of the BSD 3-Clause License
 * @date 2020
 */

#include <iostream>

// Catch2
#include <catch2/catch_test_macros.hpp>

#include <QpSolversEigen/QpSolversEigen.hpp>

TEST_CASE("QPProblem - Unconstrained")
{
    constexpr double tolerance = 1e-4;

    Eigen::SparseMatrix<double> H_s(2, 2);
    H_s.insert(0, 0) = 3;
    H_s.insert(0, 1) = 2;
    H_s.insert(1, 0) = 2;
    H_s.insert(1, 1) = 4;

    Eigen::Matrix<double, 2, 1> gradient;
    gradient << 3, 1;

    QpSolversEigen::Solver solver;
    REQUIRE(solver.instantiateSolver("osqp"));
    REQUIRE(solver.setBooleanParameter("verbose", true));
    REQUIRE(solver.setRealNumberParameter("alpha", 1.0));

    solver.setNumberOfVariables(2);
    solver.setNumberOfConstraints(0);

    REQUIRE(solver.data()->setHessianMatrix(H_s));
    REQUIRE(solver.data()->setGradient(gradient));

    REQUIRE(solver.initSolver());
    REQUIRE(solver.solveProblem() == QpSolversEigen::ErrorExitFlag::NoError);

    // expected solution
    Eigen::Matrix<double, 2, 1> expectedSolution;
    expectedSolution << -1.2500, 0.3750;

    REQUIRE(solver.getSolution().isApprox(expectedSolution, tolerance));
}

TEST_CASE("QPProblem")
{
    constexpr double tolerance = 1e-4;

    Eigen::SparseMatrix<double> H_s(2, 2);
    H_s.insert(0, 0) = 4;
    H_s.insert(0, 1) = 1;
    H_s.insert(1, 0) = 1;
    H_s.insert(1, 1) = 2;

    Eigen::SparseMatrix<double> A_s(3, 2);
    A_s.insert(0, 0) = 1;
    A_s.insert(0, 1) = 1;
    A_s.insert(1, 0) = 1;
    A_s.insert(2, 1) = 1;

    Eigen::Matrix<double, 2, 1> gradient;
    gradient << 1, 1;

    Eigen::Matrix<double, 3, 1> lowerBound;
    lowerBound << 1, 0, 0;

    Eigen::Matrix<double, 3, 1> upperBound;
    upperBound << 1, 0.7, 0.7;

    QpSolversEigen::Solver solver;
    REQUIRE(solver.instantiateSolver("osqp"));
    REQUIRE(solver.setBooleanParameter("verbose", true));
    REQUIRE(solver.setRealNumberParameter("alpha", 1.0));
    // This is required to avoid non-deterministic non-accurate solutions
    // See https://github.com/robotology/osqp-eigen/pull/172
    REQUIRE(solver.setBooleanParameter("polish", true));


    REQUIRE_FALSE(solver.data()->setHessianMatrix(H_s));
    solver.data()->setNumberOfVariables(2);

    solver.data()->setNumberOfConstraints(3);
    REQUIRE(solver.data()->setHessianMatrix(H_s));
    REQUIRE(solver.data()->setGradient(gradient));
    REQUIRE(solver.data()->setLinearConstraintsMatrix(A_s));
    REQUIRE(solver.data()->setLowerBound(lowerBound));
    REQUIRE(solver.data()->setUpperBound(upperBound));

    REQUIRE(solver.initSolver());

    REQUIRE(solver.solveProblem() == QpSolversEigen::ErrorExitFlag::NoError);
    Eigen::Matrix<double, 2, 1> expectedSolution;
    expectedSolution << 0.3, 0.7;

    REQUIRE(solver.getSolution().isApprox(expectedSolution, tolerance));
}
