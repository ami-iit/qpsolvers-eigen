#include <QpSolversEigen/QpSolversEigen.hpp>
#include <Eigen/Dense>

#include <iostream>
#include <numbers>

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
    xMin << -std::numbers::pi / 6, -std::numbers::pi / 6, -QpSolversEigen::INFTY, -QpSolversEigen::INFTY, -QpSolversEigen::INFTY, -1.,
        -QpSolversEigen::INFTY, -QpSolversEigen::INFTY, -QpSolversEigen::INFTY, -QpSolversEigen::INFTY,
        -QpSolversEigen::INFTY, -QpSolversEigen::INFTY;

    xMax << std::numbers::pi / 6, std::numbers::pi / 6, QpSolversEigen::INFTY, QpSolversEigen::INFTY, QpSolversEigen::INFTY,
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
                         Eigen::VectorXd& gradient)
{

    Eigen::Matrix<double, 12, 1> Qx_ref;
    Qx_ref = Q * (-xRef);

    // populate the gradient vector
    gradient = Eigen::VectorXd::Zero(12 * (mpcWindow + 1) + 4 * mpcWindow, 1);
    for (int i = 0; i < 12 * (mpcWindow + 1); i++)
    {
        int posQ = i % 12;
        float value = Qx_ref(posQ, 0);
        gradient(i, 0) = value;
    }
}

void castMPCToQPLinearConstraintMatrix(const Eigen::Matrix<double, 12, 12>& dynamicMatrix,
                                 const Eigen::Matrix<double, 12, 4>& controlMatrix,
                                 int mpcWindow,
                                 Eigen::SparseMatrix<double>& constraintMatrix)
{
    constraintMatrix.resize( 12 * (mpcWindow + 1),
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
}

void castMPCToQPInequalityConstraintMatrix(int mpcWindow, Eigen::SparseMatrix<double>& constraintMatrix)
{
    constraintMatrix.resize(12 * (mpcWindow + 1) + 4 * mpcWindow,
                            12 * (mpcWindow + 1) + 4 * mpcWindow);

    // populate linear constraint matrix
    for (int i = 0; i < 12 * (mpcWindow + 1) + 4 * mpcWindow; i++)
    {
        constraintMatrix.insert(i, i) = 1;
    }
}

void castMPCToQPConstraintVectors(const Eigen::Matrix<double, 12, 1>& xMax,
                                  const Eigen::Matrix<double, 12, 1>& xMin,
                                  const Eigen::Matrix<double, 4, 1>& uMax,
                                  const Eigen::Matrix<double, 4, 1>& uMin,
                                  const Eigen::Matrix<double, 12, 1>& x0,
                                  int mpcWindow,
                                  Eigen::VectorXd& equalityVectorConstraints,
                                  Eigen::VectorXd& lowerInequality,
                                  Eigen::VectorXd& upperInequality)
{
    // evaluate the lower and the upper inequality vectors
    lowerInequality
        = Eigen::MatrixXd::Zero(12 * (mpcWindow + 1) + 4 * mpcWindow, 1);
    upperInequality
        = Eigen::MatrixXd::Zero(12 * (mpcWindow + 1) + 4 * mpcWindow, 1);
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
    equalityVectorConstraints = Eigen::MatrixXd::Zero(12 * (mpcWindow + 1), 1);
    equalityVectorConstraints.block(0, 0, 12, 1) = -x0;
}

void updateLinearConstraintVectors(const Eigen::Matrix<double, 12, 1>& x0,
                             Eigen::VectorXd& equalityVectorConstraints)
{
    equalityVectorConstraints.block(0, 0, 12, 1) = -x0;
}

double getErrorNorm(const Eigen::Matrix<double, 12, 1>& x, const Eigen::Matrix<double, 12, 1>& xRef)
{
    // evaluate the error
    Eigen::Matrix<double, 12, 1> error = x - xRef;

    // return the norm
    return error.norm();
}

int main()
{
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
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearEqMatrix;
    Eigen::SparseMatrix<double> linearInqeMatrix;
    Eigen::VectorXd equalityVectorConstraints;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

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
    castMPCToQPLinearConstraintMatrix(a, b, mpcWindow, linearEqMatrix);
    castMPCToQPInequalityConstraintMatrix(mpcWindow, linearInqeMatrix);
    castMPCToQPConstraintVectors(xMax, xMin, uMax, uMin, x0, mpcWindow, equalityVectorConstraints, lowerBound, upperBound);

    // instantiate the solver
    QpSolversEigen::Solver solver;
    bool ok = solver.instantiateSolver("proxqp");
    if (!ok)
    {
        std::cerr << "QpSolverEigenMPCExample: Failed to instantiate the solver." << std::endl;
        return EXIT_FAILURE;
    }

    // settings

    // solver.setBooleanParameter("verbose", false);

    // Set osqp-specific parameters
    if (solver.getSolverName() == "osqp")
    {
        solver.setBooleanParameter("warm_starting", true);
    }

    // Set proxqp-specific parameters
    if (solver.getSolverName() == "proxqp")
    {
        solver.setBooleanParameter("verbose", true);
        solver.setBooleanParameter("compute_timings", true);
        solver.setStringParameter("initial_guess", "WARM_START_WITH_PREVIOUS_RESULT");
    }

    // set the initial data of the QP solver
    solver.setNumberOfVariables(12 * (mpcWindow + 1) + 4 * mpcWindow);
    solver.setNumberOfInequalityConstraints(linearInqeMatrix.rows());
    solver.setNumberOfEqualityConstraints(linearEqMatrix.rows());

    if (!solver.setHessianMatrix(hessian))
        return 1;
    if (!solver.setGradient(gradient))
        return 1;
    if (!solver.setInequalityConstraintsMatrix(linearInqeMatrix))
        return 1;
    if (!solver.setEqualityConstraintsMatrix(linearEqMatrix))
        return 1;
    if (!solver.setEqualityConstraintsVector(equalityVectorConstraints))
        return 1;
    if (!solver.setLowerBound(lowerBound))
        return 1;
    if (!solver.setUpperBound(upperBound))
        return 1;

    // instantiate the solver
    if (!solver.initSolver())
        return 1;

    // controller input and QPSolution vector
    Eigen::Vector4d ctr;
    Eigen::VectorXd QPSolution;

    // number of iteration steps
    int numberOfSteps = 50;

    for (int i = 0; i < numberOfSteps; i++)
    {

        // solve the QP problem
        if (solver.solveProblem() != QpSolversEigen::ErrorExitFlag::NoError)
            return 1;

        // get the controller input
        QPSolution = solver.getSolution();
        ctr = QPSolution.block(12 * (mpcWindow + 1), 0, 4, 1);

        // save data into file
        auto x0Data = x0.data();

        // propagate the model
        x0 = a * x0 + b * ctr;

        // update the constraint bound
        updateLinearConstraintVectors(x0, equalityVectorConstraints);
        if (!solver.updateEqualityConstraintsVector(equalityVectorConstraints))
            return 1;
    }
    return 0;
}
