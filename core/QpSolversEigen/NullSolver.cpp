
#include <QpSolversEigen/NullSolver.hpp>

namespace QpSolversEigen
{

std::string NullSolver::getSolverName() const
{
    return "null";
}

bool NullSolver::initSolver()
{
    return false;
}

bool NullSolver::isInitialized()
{
    return false;
}

void NullSolver::clearSolver()
{
    return;
}

bool NullSolver::clearSolverVariables()
{
    return false;
}

QpSolversEigen::ErrorExitFlag NullSolver::solveProblem()
{
    return QpSolversEigen::ErrorExitFlag::SolverSpecificUnknownError;
}

QpSolversEigen::Status NullSolver::getStatus() const
{
    return QpSolversEigen::Status::SolverSpecificUnknownStatus;
}

const double NullSolver::getObjValue() const
{
    return -std::numeric_limits<double>::infinity();
}

const Eigen::Matrix<double, Eigen::Dynamic, 1>& NullSolver::getSolution()
{
    return m_dummy;
}

const Eigen::Matrix<double, Eigen::Dynamic, 1>& NullSolver::getDualSolution()
{
    return m_dummy;
}

bool NullSolver::updateHessianMatrix(const Eigen::SparseMatrix<double> &hessianMatrix)
{
    return false;
}

bool NullSolver::updateInequalityConstraintsMatrix(const Eigen::SparseMatrix<double> &linearConstraintsMatrix)
{
    return false;
}

bool NullSolver::updateGradient(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& gradient)
{
    return false;
}

bool NullSolver::updateLowerBound(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& lowerBound)
{
    return false;
}

bool NullSolver::updateUpperBound(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& upperBound)
{
    return false;
}

bool NullSolver::updateBounds(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& lowerBound,
             const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& upperBound)
{
    return false;
}

bool NullSolver::updateEqualityConstraintsMatrix(const Eigen::SparseMatrix<double>& equalityConstraintsMatrix)
{
    return false;
}

bool NullSolver::updateEqualityConstraintsVector(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& equalityConstraintsVector)
{
    return false;
}

void NullSolver::clearHessianMatrix()
{
    return;
}

void NullSolver::clearLinearConstraintsMatrix()
{
    return;
}

void NullSolver::setNumberOfVariables(int n)
{
    return;
}

void NullSolver::setNumberOfInequalityConstraints(int m)
{
    return;
}

void NullSolver::setNumberOfEqualityConstraints(int m)
{
    return;
}

bool NullSolver::setHessianMatrix(const Eigen::SparseMatrix<double>& hessianMatrix)
{
    return false;
}

bool NullSolver::setGradient(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> gradientVector)
{
    return false;
}

Eigen::Matrix<double, Eigen::Dynamic, 1> NullSolver::getGradient()
{
    return Eigen::Matrix<double, Eigen::Dynamic, 1>();
}

bool
NullSolver::setInequalityConstraintsMatrix(const Eigen::SparseMatrix<double>& linearConstraintsMatrix)
{
    return false;
}

bool NullSolver::setLowerBound(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> lowerBoundVector)
{
    return false;
}

bool NullSolver::setUpperBound(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> upperBoundVector)
{
    return false;
}

bool NullSolver::setBounds(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> lowerBound,
               Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> upperBound)
{
    return false;
}

bool NullSolver::setEqualityConstraintsMatrix(const Eigen::SparseMatrix<double>& equalityConstraintsMatrix)
{
    return false;
}

bool NullSolver::setEqualityConstraintsVector(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& equalityConstraintsVector)
{
    return false;
}

bool NullSolver::setBooleanParameter(const std::string& settingName, bool value)
{
    return false;
}

bool NullSolver::setIntegerParameter(const std::string& settingName, int64_t value)
{
    return false;
}

bool NullSolver::setRealNumberParameter(const std::string& settingName, double value)
{
    return false;
}

bool NullSolver::setStringParameter(const std::string& parameterName, const std::string& value)
{
    return false;
}

bool NullSolver::getBooleanParametersNames(std::vector<std::string>& parametersNames) const
{
    return false;
}

bool NullSolver::getIntegerParametersNames(std::vector<std::string>& parametersNames) const
{
    return false;
}

bool NullSolver::getRealNumberParametersNames(std::vector<std::string>& parametersNames) const
{
    return false;
}

bool NullSolver::getStringParametersNames(std::vector<std::string>& parametersNames) const
{
    return false;
}

SolverInterface* NullSolver::allocateInstance() const
{
    return new NullSolver();
}

}
