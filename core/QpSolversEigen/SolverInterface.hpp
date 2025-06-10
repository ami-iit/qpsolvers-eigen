#ifndef QPSOLVERSEIGEN_SOLVERINTERFACE_HPP
#define QPSOLVERSEIGEN_SOLVERINTERFACE_HPP

// Std
#include <memory>
#include <string>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Sparse>

// QpSolversEigen
#include <QpSolversEigen/Constants.hpp>

namespace QpSolversEigen
{
/**
 * SolverInterface class is the virtual class implemented by the solver-specific classes.
 *
 * Given the specific structure of the library, where the QpSolversEigen::Solver class 
 * is acting both as a factory and as a proxy for each solver methods, most of the methods
 * are duplicated between the Solver class and the SolverInterface class. For this
 * reason, the methods are documented only once in the QpSolversEigen::Solver class.
 */
class SolverInterface
{
public:
    virtual ~SolverInterface() = default;

    virtual std::string getSolverName() const = 0;
    virtual bool initSolver() = 0;
    virtual bool isInitialized() = 0;
    virtual void clearSolver() = 0;
    virtual bool clearSolverVariables() = 0;
    virtual QpSolversEigen::ErrorExitFlag solveProblem() = 0;
    virtual QpSolversEigen::Status getStatus() const = 0;
    virtual const double getObjValue() const = 0;
    virtual const Eigen::Matrix<double, Eigen::Dynamic, 1>& getSolution() = 0;
    virtual  const Eigen::Matrix<double, Eigen::Dynamic, 1>& getDualSolution() = 0;
    virtual bool updateHessianMatrix(const Eigen::SparseMatrix<double> &hessianMatrix) = 0;
    virtual bool updateInequalityConstraintsMatrix(const Eigen::SparseMatrix<double> &linearConstraintsMatrix) = 0;
    virtual bool updateGradient(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& gradient) = 0;
    virtual bool updateLowerBound(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& lowerBound) = 0;
    virtual bool updateUpperBound(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& upperBound) = 0;
    virtual bool updateBounds(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& lowerBound,
                 const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& upperBound) = 0;
    virtual bool updateEqualityConstraintsMatrix(const Eigen::SparseMatrix<double>& equalityConstraintsMatrix) = 0;
    virtual bool updateEqualityConstraintsVector(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& equalityConstraintsVector) = 0;

    virtual void clearHessianMatrix() = 0;
    virtual void clearLinearConstraintsMatrix() = 0;
    virtual void setNumberOfVariables(int n) = 0;
    virtual void setNumberOfInequalityConstraints(int m) = 0;
    virtual void setNumberOfEqualityConstraints(int m) = 0;
    virtual bool setHessianMatrix(const Eigen::SparseMatrix<double>& hessianMatrix) = 0;
    virtual bool setGradient(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> gradientVector) = 0;

    virtual Eigen::Matrix<double, Eigen::Dynamic, 1> getGradient() = 0;

    virtual bool
    setInequalityConstraintsMatrix(const Eigen::SparseMatrix<double>& linearConstraintsMatrix) = 0;
    virtual bool setLowerBound(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> lowerBoundVector) = 0;
    virtual bool setUpperBound(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> upperBoundVector) = 0;
    virtual bool setBounds(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> lowerBound,
                   Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> upperBound) = 0;
    virtual bool setEqualityConstraintsMatrix(const Eigen::SparseMatrix<double>& equalityConstraintsMatrix) = 0;
    virtual bool setEqualityConstraintsVector(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& equalityConstraintsVector) = 0;

    virtual bool setBooleanParameter(const std::string& settingName, bool value) = 0;
    virtual bool setIntegerParameter(const std::string& settingName, int64_t value) = 0;
    virtual bool setRealNumberParameter(const std::string& settingName, double value) = 0;
    virtual bool setStringParameter(const std::string& settingName, const std::string& value) = 0;

    virtual bool getBooleanParametersNames(std::vector<std::string>& parametersNames) const = 0;
    virtual bool getIntegerParametersNames(std::vector<std::string>& parameterNames) const = 0;
    virtual bool getRealNumberParametersNames(std::vector<std::string>& parametersNames) const = 0;
    virtual bool getStringParametersNames(std::vector<std::string>& parametersNames) const = 0;

    /**
     * Allocate a new instance of this class, and return a pointer to it.
     * The ownership of the pointer is transferred to the caller.
     */
    virtual SolverInterface* allocateInstance() const = 0;
};

}

#endif
