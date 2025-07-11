/**
 * @file MPCTest.cpp
 * @author Giulio Romualdi
 * @copyright Released under the terms of the BSD 3-Clause License
 * @date 2018-2024
 */

// Include macros common to all test
#include "QpSolversEigenCommonTestMacros.hpp"

// Catch2
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators_all.hpp>

// QpSolversEigen
#include <QpSolversEigen/QpSolversEigen.hpp>

// eigen
#include <Eigen/Dense>

#include <cmath>
#include <fstream>
#include <iostream>

// colors
#define ANSI_TXT_GRN "\033[0;32m"
#define ANSI_TXT_MGT "\033[0;35m" // Magenta
#define ANSI_TXT_DFT "\033[0;0m" // Console default
#define GTEST_BOX "[     cout ] "
#define COUT_GTEST ANSI_TXT_GRN << GTEST_BOX // You could add the Default
#define COUT_GTEST_MGT COUT_GTEST << ANSI_TXT_MGT

void setDynamicsMatrices(Eigen::Matrix<double, 12, 12>& a, Eigen::Matrix<double, 12, 4>& b)
{
    a << 1., 0., 0., 0., 0., 0., 0.1, 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.1, 0., 0.,
        0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.1, 0., 0., 0., 0.0488, 0., 0., 1., 0., 0., 0.0016,
        0., 0., 0.0992, 0., 0., 0., -0.0488, 0., 0., 1., 0., 0., -0.0016, 0., 0., 0.0992, 0., 0.,
        0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.0992, 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0.,
        0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0.,
        0., 0., 0.9734, 0., 0., 0., 0., 0., 0.0488, 0., 0., 0.9846, 0., 0., 0., -0.9734, 0., 0., 0.,
        0., 0., -0.0488, 0., 0., 0.9846, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.9846;

    b << 0., -0.0726, 0., 0.0726, -0.0726, 0., 0.0726, 0., -0.0152, 0.0152, -0.0152, 0.0152, -0.,
        -0.0006, -0., 0.0006, 0.0006, 0., -0.0006, 0.0000, 0.0106, 0.0106, 0.0106, 0.0106, 0,
        -1.4512, 0., 1.4512, -1.4512, 0., 1.4512, 0., -0.3049, 0.3049, -0.3049, 0.3049, -0.,
        -0.0236, 0., 0.0236, 0.0236, 0., -0.0236, 0., 0.2107, 0.2107, 0.2107, 0.2107;
}

void setInequalityConstraints(Eigen::Matrix<double, 12, 1>& xMax,
                              Eigen::Matrix<double, 12, 1>& xMin,
                              Eigen::Matrix<double, 4, 1>& uMax,
                              Eigen::Matrix<double, 4, 1>& uMin)
{
    double u0 = 10.5916;

    // input inequality constraints
    uMin << 9.6 - u0, 9.6 - u0, 9.6 - u0, 9.6 - u0;

    uMax << 13 - u0, 13 - u0, 13 - u0, 13 - u0;

    // state inequality constraints
    xMin << -M_PI / 6, -M_PI / 6, -QpSolversEigen::INFTY, -QpSolversEigen::INFTY, -QpSolversEigen::INFTY, -1.,
        -QpSolversEigen::INFTY, -QpSolversEigen::INFTY, -QpSolversEigen::INFTY, -QpSolversEigen::INFTY,
        -QpSolversEigen::INFTY, -QpSolversEigen::INFTY;

    xMax << M_PI / 6, M_PI / 6, QpSolversEigen::INFTY, QpSolversEigen::INFTY, QpSolversEigen::INFTY,
        QpSolversEigen::INFTY, QpSolversEigen::INFTY, QpSolversEigen::INFTY, QpSolversEigen::INFTY, QpSolversEigen::INFTY,
        QpSolversEigen::INFTY, QpSolversEigen::INFTY;
}

void setWeightMatrices(Eigen::DiagonalMatrix<double, 12>& Q, Eigen::DiagonalMatrix<double, 4>& R)
{
    Q.diagonal() << 0, 0, 10., 10., 10., 10., 0, 0, 0, 5., 5., 5.;
    R.diagonal() << 0.1, 0.1, 0.1, 0.1;
}

void castMPCToQPHessian(const Eigen::DiagonalMatrix<double, 12>& Q,
                        const Eigen::DiagonalMatrix<double, 4>& R,
                        int mpcWindow,
                        Eigen::SparseMatrix<double>& hessianMatrix)
{

    hessianMatrix.resize(12 * (mpcWindow + 1) + 4 * mpcWindow,
                         12 * (mpcWindow + 1) + 4 * mpcWindow);

    // populate hessian matrix
    for (int i = 0; i < 12 * (mpcWindow + 1) + 4 * mpcWindow; i++)
    {
        if (i < 12 * (mpcWindow + 1))
        {
            int posQ = i % 12;
            float value = Q.diagonal()[posQ];
            if (value != 0)
                hessianMatrix.insert(i, i) = value;
        } else
        {
            int posR = i % 4;
            float value = R.diagonal()[posR];
            if (value != 0)
                hessianMatrix.insert(i, i) = value;
        }
    }
}

void castMPCToQPGradient(const Eigen::DiagonalMatrix<double, 12>& Q,
                         const Eigen::Matrix<double, 12, 1>& xRef,
                         int mpcWindow,
                         Eigen::Matrix<double, -1, 1>& gradient)
{

    Eigen::Matrix<double, 12, 1> Qx_ref;
    Qx_ref = Q * (-xRef);

    // populate the gradient vector
    gradient = Eigen::Matrix<double, -1, 1>::Zero(12 * (mpcWindow + 1) + 4 * mpcWindow, 1);
    for (int i = 0; i < 12 * (mpcWindow + 1); i++)
    {
        int posQ = i % 12;
        float value = Qx_ref(posQ, 0);
        gradient(i, 0) = value;
    }
}

void castMPCToQPConstraintMatrix(const Eigen::Matrix<double, 12, 12>& dynamicMatrix,
                                 const Eigen::Matrix<double, 12, 4>& controlMatrix,
                                 int mpcWindow,
                                 Eigen::SparseMatrix<double>& constraintMatrix)
{
    constraintMatrix.resize(12 * (mpcWindow + 1) + 12 * (mpcWindow + 1) + 4 * mpcWindow,
                            12 * (mpcWindow + 1) + 4 * mpcWindow);

    // populate linear constraint matrix
    for (int i = 0; i < 12 * (mpcWindow + 1); i++)
    {
        constraintMatrix.insert(i, i) = -1;
    }

    for (int i = 0; i < mpcWindow; i++)
        for (int j = 0; j < 12; j++)
            for (int k = 0; k < 12; k++)
            {
                float value = dynamicMatrix(j, k);
                if (value != 0)
                {
                    constraintMatrix.insert(12 * (i + 1) + j, 12 * i + k) = value;
                }
            }

    for (int i = 0; i < mpcWindow; i++)
        for (int j = 0; j < 12; j++)
            for (int k = 0; k < 4; k++)
            {
                float value = controlMatrix(j, k);
                if (value != 0)
                {
                    constraintMatrix.insert(12 * (i + 1) + j, 4 * i + k + 12 * (mpcWindow + 1))
                        = value;
                }
            }

    for (int i = 0; i < 12 * (mpcWindow + 1) + 4 * mpcWindow; i++)
    {
        constraintMatrix.insert(i + (mpcWindow + 1) * 12, i) = 1;
    }
}

void castMPCToQPConstraintVectors(const Eigen::Matrix<double, 12, 1>& xMax,
                                  const Eigen::Matrix<double, 12, 1>& xMin,
                                  const Eigen::Matrix<double, 4, 1>& uMax,
                                  const Eigen::Matrix<double, 4, 1>& uMin,
                                  const Eigen::Matrix<double, 12, 1>& x0,
                                  int mpcWindow,
                                  Eigen::Matrix<double, -1, 1>& lowerBound,
                                  Eigen::Matrix<double, -1, 1>& upperBound)
{
    // evaluate the lower and the upper inequality vectors
    Eigen::Matrix<double, -1, 1> lowerInequality
        = Eigen::Matrix<double, -1, -1>::Zero(12 * (mpcWindow + 1) + 4 * mpcWindow, 1);
    Eigen::Matrix<double, -1, 1> upperInequality
        = Eigen::Matrix<double, -1, -1>::Zero(12 * (mpcWindow + 1) + 4 * mpcWindow, 1);
    for (int i = 0; i < mpcWindow + 1; i++)
    {
        lowerInequality.block(12 * i, 0, 12, 1) = xMin;
        upperInequality.block(12 * i, 0, 12, 1) = xMax;
    }
    for (int i = 0; i < mpcWindow; i++)
    {
        lowerInequality.block(4 * i + 12 * (mpcWindow + 1), 0, 4, 1) = uMin;
        upperInequality.block(4 * i + 12 * (mpcWindow + 1), 0, 4, 1) = uMax;
    }

    // evaluate the lower and the upper equality vectors
    Eigen::Matrix<double, -1, 1> lowerEquality
        = Eigen::Matrix<double, -1, -1>::Zero(12 * (mpcWindow + 1), 1);
    Eigen::Matrix<double, -1, 1> upperEquality;
    lowerEquality.block(0, 0, 12, 1) = -x0;
    upperEquality = lowerEquality;
    lowerEquality = lowerEquality;

    // merge inequality and equality vectors
    lowerBound = Eigen::Matrix<double, -1, -1>::Zero(2 * 12 * (mpcWindow + 1) + 4 * mpcWindow, 1);
    lowerBound << lowerEquality, lowerInequality;

    upperBound = Eigen::Matrix<double, -1, -1>::Zero(2 * 12 * (mpcWindow + 1) + 4 * mpcWindow, 1);
    upperBound << upperEquality, upperInequality;
}

void updateConstraintVectors(const Eigen::Matrix<double, 12, 1>& x0,
                             Eigen::Matrix<double, -1, 1>& lowerBound,
                             Eigen::Matrix<double, -1, 1>& upperBound)
{
    lowerBound.block(0, 0, 12, 1) = -x0;
    upperBound.block(0, 0, 12, 1) = -x0;
}

double
getErrorNorm(const Eigen::Matrix<double, 12, 1>& x, const Eigen::Matrix<double, 12, 1>& xRef)
{
    // evaluate the error
    Eigen::Matrix<double, 12, 1> error = x - xRef;

    // return the norm
    return error.norm();
}

TEST_CASE("MPCTest")
{
    // open the ofstream
    std::ofstream dataStream;
    dataStream.open("output.txt");

    // set the preview window
    int mpcWindow = 20;

    // allocate the dynamics matrices
    Eigen::Matrix<double, 12, 12> a;
    Eigen::Matrix<double, 12, 4> b;

    // allocate the constraints vector
    Eigen::Matrix<double, 12, 1> xMax;
    Eigen::Matrix<double, 12, 1> xMin;
    Eigen::Matrix<double, 4, 1> uMax;
    Eigen::Matrix<double, 4, 1> uMin;

    // allocate the weight matrices
    Eigen::DiagonalMatrix<double, 12> Q;
    Eigen::DiagonalMatrix<double, 4> R;

    // allocate the initial and the reference state space
    Eigen::Matrix<double, 12, 1> x0;
    Eigen::Matrix<double, 12, 1> xRef;

    // allocate QP problem matrices and vectors
    Eigen::SparseMatrix<double> hessian;
    Eigen::Matrix<double, -1, 1> gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::Matrix<double, -1, 1> lowerBound;
    Eigen::Matrix<double, -1, 1> upperBound;


    // set the initial and the desired states
    x0 << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    xRef << 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    // set MPC problem quantities
    setDynamicsMatrices(a, b);
    setInequalityConstraints(xMax, xMin, uMax, uMin);
    setWeightMatrices(Q, R);

    // cast the MPC problem as QP problem
    castMPCToQPHessian(Q, R, mpcWindow, hessian);
    castMPCToQPGradient(Q, xRef, mpcWindow, gradient);
    castMPCToQPConstraintMatrix(a, b, mpcWindow, linearMatrix);
    castMPCToQPConstraintVectors(xMax, xMin, uMax, uMin, x0, mpcWindow, lowerBound, upperBound);


    // instantiate the solver
    std::string solverName = QPSOLVERSEIGEN_SOLVERS_TO_TEST;
    QpSolversEigen::Solver solver;
    REQUIRE(solver.instantiateSolver(solverName));

    std::cout << COUT_GTEST_MGT << "Testing solver " << solver.getSolverName()
              << ANSI_TXT_DFT << std::endl;

    // settings
    REQUIRE(solver.setBooleanParameter("verbose", false));

    // Set solver-specific parameters
    if (solver.getSolverName() == "osqp")
    {
        REQUIRE(solver.setBooleanParameter("warm_start", true));
    }
    if (solver.getSolverName() == "proxqp")
    {
        REQUIRE(solver.setStringParameter("initial_guess", "WARM_START_WITH_PREVIOUS_RESULT"));
    }

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(12 * (mpcWindow + 1) + 4 * mpcWindow);
    solver.data()->setNumberOfInequalityConstraints(2 * 12 * (mpcWindow + 1) + 4 * mpcWindow);
    REQUIRE(solver.data()->setHessianMatrix(hessian));
    REQUIRE(solver.data()->setGradient(gradient));
    REQUIRE(solver.data()->setInequalityConstraintsMatrix(linearMatrix));
    REQUIRE(solver.data()->setLowerBound(lowerBound));
    REQUIRE(solver.data()->setUpperBound(upperBound));

    // instantiate the solver
    REQUIRE(solver.initSolver());

    // controller input and QPSolution vector
    Eigen::Matrix<double, 4, 1> ctr;
    Eigen::Matrix<double, -1, 1> QPSolution;

    // number of iteration steps
    int numberOfSteps = 50;

    // profiling quantities
    clock_t startTime, endTime;
    double averageTime = 0;

    for (int i = 0; i < numberOfSteps; i++)
    {
        startTime = clock();

        // solve the QP problem
        REQUIRE(solver.solveProblem() == QpSolversEigen::ErrorExitFlag::NoError);

        // get the controller input
        QPSolution = solver.getSolution();
        ctr = QPSolution.block(12 * (mpcWindow + 1), 0, 4, 1);

        // save data into file
        auto x0Data = x0.data();
        for (int j = 0; j < 12; j++)
            dataStream << x0Data[j] << " ";
        dataStream << std::endl;

        // propagate the model
        x0 = a * x0 + b * ctr;

        // update the constraint bound
        updateConstraintVectors(x0, lowerBound, upperBound);
        REQUIRE(solver.updateBounds(lowerBound, upperBound));

        endTime = clock();

        averageTime += static_cast<double>(endTime - startTime) / CLOCKS_PER_SEC;
    }

    // close the stream
    dataStream.close();

    std::cout << COUT_GTEST_MGT << "Average time = " << averageTime / numberOfSteps << " seconds."
              << ANSI_TXT_DFT << std::endl;

    constexpr double tolerance = 1e-2;
    REQUIRE(getErrorNorm(x0, xRef) <= tolerance);
}
