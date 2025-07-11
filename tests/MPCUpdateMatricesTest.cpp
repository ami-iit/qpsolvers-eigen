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

#define T 0.1

void setDynamicsMatrices(Eigen::Matrix<double, 2, 2>& a,
                         Eigen::Matrix<double, 2, 1>& b,
                         Eigen::Matrix<double, 1, 2>& c,
                         double t)
{

    double omega = 0.1132;
    double alpha = 0.5 * sin(2 * M_PI * omega * t);
    double beta = 2 - 1 * sin(2 * M_PI * omega * t);

    a << alpha, 1, 0, alpha;

    b << 0, 1;

    c << beta, 0;
}

void setWeightMatrices(Eigen::DiagonalMatrix<double, 1>& Q, Eigen::DiagonalMatrix<double, 1>& R)
{
    Q.diagonal() << 10;
    R.diagonal() << 1;
}

void castMPCToQPHessian(const Eigen::DiagonalMatrix<double, 1>& Q,
                        const Eigen::DiagonalMatrix<double, 1>& R,
                        int mpcWindow,
                        int k,
                        Eigen::SparseMatrix<double>& hessianMatrix)
{

    Eigen::Matrix<double, 2, 2> a;
    Eigen::Matrix<double, 2, 1> b;
    Eigen::Matrix<double, 1, 2> c;

    hessianMatrix.resize(2 * (mpcWindow + 1) + 1 * mpcWindow, 2 * (mpcWindow + 1) + 1 * mpcWindow);

    // populate hessian matrix
    for (int i = 0; i < 2 * (mpcWindow + 1) + 1 * mpcWindow; i++)
    {
        double t = (k + i) * T;
        setDynamicsMatrices(a, b, c, t);
        if (i < 2 * (mpcWindow + 1))
        {
            // here the structure of the matrix c is used!
            int pos = i % 2;
            float value = c(pos) * Q.diagonal()[0] * c(pos);
            if (value != 0)
                hessianMatrix.insert(i, i) = value;
        } else
        {
            float value = R.diagonal()[0];
            if (value != 0)
                hessianMatrix.insert(i, i) = value;
        }
    }
}

void castMPCToQPGradient(const Eigen::DiagonalMatrix<double, 1>& Q,
                         const Eigen::Matrix<double, 1, 1>& yRef,
                         int mpcWindow,
                         int k,
                         Eigen::Matrix<double, -1, 1>& gradient)
{

    Eigen::Matrix<double, 2, 2> a;
    Eigen::Matrix<double, 2, 1> b;
    Eigen::Matrix<double, 1, 2> c;

    Eigen::Matrix<double, 1, 1> Qy_ref;
    Qy_ref = Q * (-yRef);

    // populate the gradient vector
    gradient = Eigen::Matrix<double, -1, 1>::Zero(2 * (mpcWindow + 1) + 1 * mpcWindow, 1);
    for (int i = 0; i < 2 * (mpcWindow + 1); i++)
    {
        double t = (k + i) * T;
        setDynamicsMatrices(a, b, c, t);

        int pos = i % 2;
        float value = Qy_ref(0, 0) * c(pos);
        gradient(i, 0) = value;
    }
}

void castMPCToQPConstraintMatrix(int mpcWindow,
                                 int k,
                                 Eigen::SparseMatrix<double>& constraintMatrix)
{
    constraintMatrix.resize(2 * (mpcWindow + 1), 2 * (mpcWindow + 1) + 1 * mpcWindow);

    // populate linear constraint matrix
    for (int i = 0; i < 2 * (mpcWindow + 1); i++)
    {
        constraintMatrix.insert(i, i) = -1;
    }

    Eigen::Matrix<double, 2, 2> a;
    Eigen::Matrix<double, 2, 1> b;
    Eigen::Matrix<double, 1, 2> c;

    for (int i = 0; i < mpcWindow; i++)
    {
        double t = (k + i) * T;
        setDynamicsMatrices(a, b, c, t);
        for (int j = 0; j < 2; j++)
            for (int k = 0; k < 2; k++)
            {
                float value = a(j, k);
                if (value != 0)
                {
                    constraintMatrix.insert(2 * (i + 1) + j, 2 * i + k) = value;
                }
            }
    }

    for (int i = 0; i < mpcWindow; i++)
        for (int j = 0; j < 2; j++)
            for (int k = 0; k < 1; k++)
            {
                // b is constant
                float value = b(j, k);
                if (value != 0)
                {
                    constraintMatrix.insert(2 * (i + 1) + j, 1 * i + k + 2 * (mpcWindow + 1))
                        = value;
                }
            }
}

void castMPCToQPConstraintVectors(const Eigen::Matrix<double, 2, 1>& x0,
                                  int mpcWindow,
                                  Eigen::Matrix<double, -1, 1>& lowerBound,
                                  Eigen::Matrix<double, -1, 1>& upperBound)
{
    // evaluate the lower and the upper equality vectors
    lowerBound = Eigen::Matrix<double, -1, -1>::Zero(2 * (mpcWindow + 1), 1);
    lowerBound.block(0, 0, 2, 1) = -x0;
    upperBound = lowerBound;
}

bool updateHessianMatrix(QpSolversEigen::Solver& solver,
                         const Eigen::DiagonalMatrix<double, 1>& Q,
                         const Eigen::DiagonalMatrix<double, 1>& R,
                         int mpcWindow,
                         int k)
{
    Eigen::SparseMatrix<double> hessianMatrix;
    castMPCToQPHessian(Q, R, mpcWindow, k, hessianMatrix);

    if (!solver.updateHessianMatrix(hessianMatrix))
        return false;

    return true;
}

bool updateLinearConstraintsMatrix(QpSolversEigen::Solver& solver, int mpcWindow, int k)
{
    Eigen::SparseMatrix<double> constraintMatrix;
    castMPCToQPConstraintMatrix(mpcWindow, k, constraintMatrix);

    if (!solver.updateInequalityConstraintsMatrix(constraintMatrix))
        return false;

    return true;
}

void updateConstraintVectors(const Eigen::Matrix<double, 2, 1>& x0,
                             Eigen::Matrix<double, -1, 1>& lowerBound,
                             Eigen::Matrix<double, -1, 1>& upperBound)
{
    lowerBound.block(0, 0, 2, 1) = -x0;
    upperBound.block(0, 0, 2, 1) = -x0;
}

TEST_CASE("MPCTest Update matrices")
{
    // open the ofstream
    std::ofstream dataStream;
    dataStream.open("output.txt");

    // set the preview window
    int mpcWindow = 100;

    // allocate the dynamics matrices
    Eigen::Matrix<double, 2, 2> a;
    Eigen::Matrix<double, 2, 1> b;
    Eigen::Matrix<double, 1, 2> c;

    // allocate the weight matrices
    Eigen::DiagonalMatrix<double, 1> Q;
    Eigen::DiagonalMatrix<double, 1> R;

    // allocate the initial and the reference state space
    Eigen::Matrix<double, 2, 1> x0;
    Eigen::Matrix<double, 1, 1> yRef;
    Eigen::Matrix<double, 1, 1> y;

    // allocate QP problem matrices and vectors
    Eigen::SparseMatrix<double> hessian;
    Eigen::Matrix<double, -1, 1> gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::Matrix<double, -1, 1> lowerBound;
    Eigen::Matrix<double, -1, 1> upperBound;

    // set the initial and the desired states
    x0 << 0, 0;
    yRef << 1;

    // set MPC problem quantities
    setWeightMatrices(Q, R);

    // cast the MPC problem as QP problem
    castMPCToQPHessian(Q, R, mpcWindow, 0, hessian);
    castMPCToQPGradient(Q, yRef, mpcWindow, 0, gradient);
    castMPCToQPConstraintMatrix(mpcWindow, 0, linearMatrix);
    castMPCToQPConstraintVectors(x0, mpcWindow, lowerBound, upperBound);

    // instantiate the solver
    std::string solverName = QPSOLVERSEIGEN_SOLVERS_TO_TEST;
    QpSolversEigen::Solver solver;
    REQUIRE(solver.instantiateSolver(solverName));
    std::cout << COUT_GTEST_MGT << "Testing solver " << solver.getSolverName()
              << ANSI_TXT_DFT << std::endl;
    // settings
    REQUIRE(solver.setBooleanParameter("verbose", false));

    if (solver.getSolverName() == "osqp")
    {
        REQUIRE(solver.setBooleanParameter("warm_start", true));
    }

    if (solver.getSolverName() == "proxqp")
    {
        REQUIRE(solver.setStringParameter("initial_guess", "WARM_START_WITH_PREVIOUS_RESULT"));
        // Check that setStringParameter fail for unknown setting or unknown value
        REQUIRE_FALSE(solver.setStringParameter("initial_guess", "THIS_IS_NOT_A_VALID_INITIAL_GUESS_VALUE"));
        REQUIRE_FALSE(solver.setStringParameter("this_is_not_a_valid_proqp_parameter_name", "THIS_IS_NOT_A_VALID_INITIAL_GUESS_VALUE"));
    }

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(2 * (mpcWindow + 1) + 1 * mpcWindow);
    solver.data()->setNumberOfInequalityConstraints(2 * (mpcWindow + 1));
    REQUIRE(solver.data()->setHessianMatrix(hessian));
    REQUIRE(solver.data()->setGradient(gradient));
    REQUIRE(solver.data()->setInequalityConstraintsMatrix(linearMatrix));
    REQUIRE(solver.data()->setLowerBound(lowerBound));
    REQUIRE(solver.data()->setUpperBound(upperBound));

    // instantiate the solver
    REQUIRE(solver.initSolver());

    // controller input and QPSolution vector
    Eigen::Matrix<double, -1, 1> ctr;
    Eigen::Matrix<double, -1, 1> QPSolution;

    // number of iteration steps
    int numberOfSteps = 50;

    // profiling quantities
    clock_t startTime, endTime;
    double averageTime = 0;

    for (int i = 0; i < numberOfSteps; i++)
    {
        startTime = clock();

        setDynamicsMatrices(a, b, c, i * T);

        // update the constraint bound
        REQUIRE(updateHessianMatrix(solver, Q, R, mpcWindow, i));
        REQUIRE(updateLinearConstraintsMatrix(solver, mpcWindow, i));

        castMPCToQPGradient(Q, yRef, mpcWindow, i, gradient);
        REQUIRE(solver.updateGradient(gradient));

        updateConstraintVectors(x0, lowerBound, upperBound);
        REQUIRE(solver.updateBounds(lowerBound, upperBound));

        // solve the QP problem
        REQUIRE(solver.solveProblem() == QpSolversEigen::ErrorExitFlag::NoError);

        // get the controller input
        QPSolution = solver.getSolution();
        ctr = QPSolution.block(2 * (mpcWindow + 1), 0, 1, 1);

        // save data into file
        auto x0Data = x0.data();
        for (int j = 0; j < 2; j++)
            dataStream << x0Data[j] << " ";
        dataStream << std::endl;

        // propagate the model
        x0 = a * x0 + b * ctr;
        y = c * x0;

        endTime = clock();

        averageTime += static_cast<double>(endTime - startTime) / CLOCKS_PER_SEC;
    }

    // close the stream
    dataStream.close();

    std::cout << COUT_GTEST_MGT << "Average time = " << averageTime / numberOfSteps << " seconds."
              << ANSI_TXT_DFT << std::endl;
}
