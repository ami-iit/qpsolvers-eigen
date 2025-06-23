#include <cstdlib>
#include <iostream>

#include <QpSolversEigen/QpSolversEigen.hpp>
#include <Eigen/Dense>
#include <Eigen/Sparse>


int main()
{
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

    // Here you select the solver, possible options are:
    // * osqp
    // * proxqp
    bool ok = solver.instantiateSolver("proxqp");

    if (!ok)
    {
        std::cerr << "Error in instantiating the solver" << std::endl;
        return EXIT_FAILURE;
    }

    // Set osqp-specific parameters
    if (solver.getSolverName() == "osqp")
    {
        solver.setBooleanParameter("verbose", true);
        solver.setRealNumberParameter("alpha", 1.0);
        // See https://github.com/robotology/osqp-eigen/pull/172
        solver.setBooleanParameter("polish", true);
    }

    solver.data()->setNumberOfVariables(2);
    solver.data()->setNumberOfInequalityConstraints(3);
    ok = ok && solver.data()->setHessianMatrix(H_s);
    ok = ok && solver.data()->setGradient(gradient);
    ok = ok && solver.data()->setInequalityConstraintsMatrix(A_s);
    ok = ok && solver.data()->setLowerBound(lowerBound);
    ok = ok && solver.data()->setUpperBound(upperBound);
    ok = ok && solver.initSolver();

    if (!ok)
    {
        std::cerr << "Error in solver initialization" << std::endl;
        return EXIT_FAILURE;
    }

    ok = ok && (solver.solveProblem() == QpSolversEigen::ErrorExitFlag::NoError);

    if (!ok)
    {
        std::cerr << "Error in solving the problem" << std::endl;
        return EXIT_FAILURE;
    }

    std::cerr << "Solution: " << solver.getSolution() << std::endl;
}
