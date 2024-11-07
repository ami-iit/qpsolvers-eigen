#ifndef QPSOLVERSEIGEN_SOLVER_HPP
#define QPSOLVERSEIGEN_SOLVER_HPP

// Std
#include <memory>
#include <string>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Sparse>

// QpSolversEigen
#include <QpSolversEigen/Constants.hpp>
#include <QpSolversEigen/SolverInterface.hpp>

namespace QpSolversEigen
{
/**
 * Solver class is the main class used to wrap the QP solvers.
 *
 * The class is used to both instantiate the solver and then interact with it.
 */
class Solver
{
public:
    /**
     * Constructor.
     */
    Solver();

    /**
     * Destructor. It is required by the pimpl implementation.
     */
    ~Solver();

    /**
     * Instantiate the solver. This is compulsory before calling any other method.
     * @param[in] solverName is the name of the solver to be used (default: osqp)
     * @return true/false in case of success/failure.
     */
    bool instantiateSolver(std::string solverName = "osqp");

    /**
     * Get the name of the solver.
     *
     * @return the value passed to the instantiateSolver method if the instantiateSolver was successful, or "null" otherwise.
     */
    std::string getSolverName() const;

    /**
     * Initialize the solver with the actual initial data and settings.
     * @return true/false in case of success/failure.
     */
    bool initSolver();

    /**
     * Check if the solver is initialized.
     * @return true if the solver is initialized.
     */
    bool isInitialized();

    /**
     * Deallocate memory.
     */
    void clearSolver();

    /**
     * Set to zero all the solver variables.
     * @return true/false in case of success/failure.
     */
    bool clearSolverVariables();

    /**
     * Solve the QP optimization problem.
     * @return the error exit flag
     */
    QpSolversEigen::ErrorExitFlag solveProblem();

    /**
     * Get the status of the solver
     * @return The inner solver status
     */
    QpSolversEigen::Status getStatus() const;

    /**
     * Get the primal objective value
     * @return The primal objective value
     */
    const double getObjValue() const;

    /**
     * Get the optimization problem solution.
     * @return an Eigen::Vector containing the optimization result.
     */
    const Eigen::Matrix<double, -1, 1>& getSolution();

    /**
     * Get the dual optimization problem solution.
     * @return an Eigen::Vector containing the optimization result.
     */
    const Eigen::Matrix<double, -1, 1>& getDualSolution();


    /**
     * Update the quadratic part of the cost function (Hessian).
     * It is assumed to be a simmetric matrix.
     * \note
     * If the sparsity pattern is preserved the matrix is simply update
     * otherwise the entire solver will be reinitialized. In this case
     * the primal and dual variable are copied in the new workspace.
     *
     * @param hessian is the Hessian matrix.
     * @return true/false in case of success/failure.
     */
    bool updateHessianMatrix(const Eigen::SparseMatrix<double> &hessianMatrix);

    /**
     * Update the linear constraints matrix (A)
     * \note
     * If the sparsity pattern is preserved the matrix is simply update
     * otherwise the entire solver will be reinitialized. In this case
     * the primal and dual variable are copied in the new workspace.
     *
     * @param linearConstraintsMatrix is the linear constraint matrix A
     * @return true/false in case of success/failure.
     */
    bool updateLinearConstraintsMatrix(const Eigen::SparseMatrix<double> &linearConstraintsMatrix);

    /**
     * Update the linear part of the cost function (Gradient).
     * @param gradient is the Gradient vector.
     * @note the elements of the gradient are not copied inside the library.
     * The user has to guarantee that the lifetime of the objects passed is the same of the
     * OsqpEigen object.
     * @return true/false in case of success/failure.
     */
    bool
    updateGradient(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& gradient);

    /**
     * Update the lower bounds limit (size m).
     * @param lowerBound is the lower bound constraint vector.
     * @note the elements of the lowerBound are not copied inside the library.
     * The user has to guarantee that the lifetime of the object passed is the same of the
     * OsqpEigen object.
     * @return true/false in case of success/failure.
     */
    bool
    updateLowerBound(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& lowerBound);

    /**
     * Update the upper bounds limit (size m).
     * @param upperBound is the upper bound constraint vector.
     * @note the elements of the upperBound are not copied inside the library.
     * The user has to guarantee that the lifetime of the object passed is the same of the
     * OsqpEigen object.
     * @return true/false in case of success/failure.
     */
    bool
    updateUpperBound(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& upperBound);

    /**
     * Update both upper and lower bounds (size m).
     * @param lowerBound is the lower bound constraint vector;
     * @param upperBound is the upper bound constraint vector.
     * @note the elements of the lowerBound and upperBound are not copied inside the library.
     * The user has to guarantee that the lifetime of the objects passed is the same of the
     * OsqpEigen object
     * @return true/false in case of success/failure.
     */
    bool
    updateBounds(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& lowerBound,
                 const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& upperBound);

    /**
     * Clear the hessian matrix.
     */
    void clearHessianMatrix();

    /**
     * Clear the linear constraints matrix.
     */
    void clearLinearConstraintsMatrix();

    /**
     * Set the number of variables.
     * @param n is the number of variables.
     */
    void setNumberOfVariables(int n);

    /**
     * Set the number of constraints.
     * @param m is the number of constraints.
     */
    void setNumberOfConstraints(int m);

    /**
     * Set the quadratic part of the cost function (Hessian).
     * It is assumed to be a symmetric matrix.
     * @param hessianMatrix is the Hessian matrix.
     * @return true/false in case of success/failure.
     */
    bool setHessianMatrix(const Eigen::SparseMatrix<double>& hessianMatrix);

    /**
     * Set the linear part of the cost function (Gradient).
     * @param gradientVector is the Gradient vector.
     * @note the elements of the gradient are not copied inside the library.
     * The user has to guarantee that the lifetime of the object passed is the same of the
     * OsqpEigen object
     * @return true/false in case of success/failure.
     */
    bool setGradient(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> gradientVector);

    Eigen::Matrix<double, Eigen::Dynamic, 1> getGradient();

    /**
     * Set the linear constraint matrix A (size m x n)
     * @param linearConstraintsMatrix is the linear constraints matrix A.
     * @return true/false in case of success/failure.
     */
    bool
    setLinearConstraintsMatrix(const Eigen::SparseMatrix<double>& linearConstraintsMatrix);

    /**
     * Set the array for lower bound (size m).
     * @param lowerBoundVector is the lower bound constraint.
     * @note the elements of the lowerBoundVector are not copied inside the library.
     * The user has to guarantee that the lifetime of the object passed is the same of the
     * OsqpEigen object
     * @return true/false in case of success/failure.
     */
    bool setLowerBound(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> lowerBoundVector);

    /**
     * Set the array for upper bound (size m).
     * @param upperBoundVector is the upper bound constraint.
     * @note the elements of the upperBoundVector are not copied inside the library.
     * The user has to guarantee that the lifetime of the object passed is the same of the
     * OsqpEigen object.
     * @return true/false in case of success/failure.
     */
    bool setUpperBound(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> upperBoundVector);

    /**
     * Set the array for upper and lower bounds (size m).
     * @param lowerBound is the lower bound constraint.
     * @param upperBound is the upper bound constraint.
     * @note the elements of the upperBound and lowerBound are not copied inside the library.
     * The user has to guarantee that the lifetime of the object passed is the same of the
     * OsqpEigen object.
     * @return true/false in case of success/failure.
     */
    bool setBounds(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> lowerBound,
                   Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> upperBound);

    /**
     * Set a boolean parameter.
     * @param parameterName the name of the parameter to bet set.
     * @return true/false in case of success/failure.
     */
    bool setBooleanParameter(const std::string& parameterName, bool value);

    /**
     * Set a integer parameter.
     * @param parameterName the name of the parameter to bet set.
     * @return true/false in case of success/failure.
     */
    bool setIntegerParameter(const std::string& parameterName, int64_t value);

    /**
     * Set a real number parameter.
     * @param parameterName the name of the parameter to bet set.
     * @return true/false in case of success/failure.
     */
    bool setRealNumberParameter(const std::string& parameterName, double value);

    /**
     * Return a pointer to this class.
     *
     * This is just for backward compatibility with OsqpEigen::Solver::data() method,
     * please do not use in QpSolversEigen-specific code.
     */
    Solver* data();

private:
    /** Private implementation */
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;
};

}

#endif
