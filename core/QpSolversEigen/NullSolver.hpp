#ifndef QPSOLVERSEIGEN_NULLSOLVER_HPP
#define QPSOLVERSEIGEN_NULLSOLVER_HPP

// Std
#include <memory>
#include <string>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Sparse>

// QpSolversEigen
#include <QpSolversEigen/SolverInterface.hpp>

namespace QpSolversEigen
{

/**
 * NullSolver class is a class that implements the SolverInterface but does nothing.
 *
 * The class is used to provide a default implementation of the SolverInterface that is used inside Solver class.
 */
class NullSolver final: public SolverInterface
{
private:
    // Dummy variable for methods that return a reference to a matrix
    Eigen::Matrix<double, Eigen::Dynamic, 1> m_dummy;
public:
    virtual ~NullSolver() = default;

    bool initSolver() override;
    bool isInitialized() override;
    void clearSolver() override;
    bool clearSolverVariables() override;
    QpSolversEigen::ErrorExitFlag solveProblem() override;
    QpSolversEigen::Status getStatus() const override;
    const double getObjValue() const override;
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& getSolution() override;
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& getDualSolution() override;
    bool updateHessianMatrix(const Eigen::SparseMatrix<double> &hessianMatrix) override;
    bool updateLinearConstraintsMatrix(const Eigen::SparseMatrix<double> &linearConstraintsMatrix) override;
    bool updateGradient(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& gradient) override;
    bool updateLowerBound(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& lowerBound) override;
    bool updateUpperBound(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& upperBound) override;
    bool updateBounds(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& lowerBound,
                 const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& upperBound) override;

    void clearHessianMatrix() override;
    void clearLinearConstraintsMatrix() override;
    void setNumberOfVariables(int n) override;
    void setNumberOfConstraints(int m) override;
    bool setHessianMatrix(const Eigen::SparseMatrix<double>& hessianMatrix) override;
    bool setGradient(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> gradientVector) override;

    Eigen::Matrix<double, Eigen::Dynamic, 1> getGradient() override;

    bool
    setLinearConstraintsMatrix(const Eigen::SparseMatrix<double>& linearConstraintsMatrix) override;
    bool setLowerBound(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> lowerBoundVector) override;
    bool setUpperBound(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> upperBoundVector) override;
    bool setBounds(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> lowerBound,
                   Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> upperBound) override;

    bool setBooleanParameter(const std::string& settingName, bool value) override;
    bool setIntegerParameter(const std::string& settingName, int64_t value) override;
    bool setRealNumberParameter(const std::string& settingName, double value) override;
    SolverInterface* allocateInstance() const override;
};

}

#endif
