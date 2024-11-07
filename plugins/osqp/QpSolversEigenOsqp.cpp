// QpSolversEigen 
#include <QpSolversEigen/SolverInterface.hpp>
#include <QpSolversEigen/Debug.hpp>

// Class factory API
#include <sharedlibpp/SharedLibraryClassApi.h>

// OsqpEigen
#include <OsqpEigen/OsqpEigen.h>

namespace QpSolversEigen
{

/**
 * ProxqpSolver class is a class that implements the SolverInterface but does nothing.
 *
 */
class ProxqpSolver final: public SolverInterface
{
private:
    // OSQP solver
    OsqpEigen::Solver osqpEigenSolver;

    // Helper class to convert OsqpEigen::ErrorExitFlag to QpSolversEigen::ErrorExitFlag
    QpSolversEigen::ErrorExitFlag convertErrorExitFlag(const OsqpEigen::ErrorExitFlag errorExitFlag) const;

    // Helper class to convert OsqpEigen::Status to QpSolversEigen::Status
    QpSolversEigen::Status convertStatus(const OsqpEigen::Status status) const;

    // Helper class to convert to convert QpSolversEigen::INFTY to OsqpEigen::INFTY in bounds
    bool convertQpSolversEigenInftyToOsqpEigenInfty(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& inputBound,
                                                    Eigen::VectorXd& outputBound);

    // Buffers used to store the bound data with infinity values compatible with OsqpEigen
    Eigen::VectorXd lowerBoundBufferWithOsqpEigenInfty;
    Eigen::VectorXd upperBoundBufferWithOsqpEigenInfty;

public:
    virtual ~ProxqpSolver() = default;

    std::string getSolverName() const override;
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

// The first argument needs to be coherent with the scheme used in
// getShlibppFactoryNameFromSolverName, i.e. qpsolvers_eigen_<solverName>
SHLIBPP_DEFINE_SHARED_SUBCLASS(qpsolvers_eigen_osqp, QpSolversEigen::ProxqpSolver, QpSolversEigen::SolverInterface);

QpSolversEigen::ErrorExitFlag ProxqpSolver::convertErrorExitFlag(const OsqpEigen::ErrorExitFlag errorExitFlag) const
{
    switch (errorExitFlag)
    {
    case OsqpEigen::ErrorExitFlag::NoError:
        return QpSolversEigen::ErrorExitFlag::NoError;
    case OsqpEigen::ErrorExitFlag::DataValidationError:
        return QpSolversEigen::ErrorExitFlag::DataValidationError;
    case OsqpEigen::ErrorExitFlag::SettingsValidationError:
        return QpSolversEigen::ErrorExitFlag::SettingsValidationError;
    case OsqpEigen::ErrorExitFlag::LinsysSolverLoadError:
        return QpSolversEigen::ErrorExitFlag::LinsysSolverLoadError;
    case OsqpEigen::ErrorExitFlag::LinsysSolverInitError:
        return QpSolversEigen::ErrorExitFlag::LinsysSolverInitError;
    case OsqpEigen::ErrorExitFlag::NonCvxError:
        return QpSolversEigen::ErrorExitFlag::NonCvxError;
    case OsqpEigen::ErrorExitFlag::MemAllocError:
        return QpSolversEigen::ErrorExitFlag::MemAllocError;
    case OsqpEigen::ErrorExitFlag::WorkspaceNotInitError:
        return QpSolversEigen::ErrorExitFlag::WorkspaceNotInitError;
    default:
        return QpSolversEigen::ErrorExitFlag::SolverSpecificUnknownError;
    }
}

QpSolversEigen::Status ProxqpSolver::convertStatus(const OsqpEigen::Status status) const
{
    switch (status)
    {
    case OsqpEigen::Status::Solved:
        return QpSolversEigen::Status::Solved;
    case OsqpEigen::Status::SolvedInaccurate:
        return QpSolversEigen::Status::SolvedInaccurate;
    case OsqpEigen::Status::PrimalInfeasible:
        return QpSolversEigen::Status::PrimalInfeasible;
    case OsqpEigen::Status::PrimalInfeasibleInaccurate:
        return QpSolversEigen::Status::PrimalInfeasibleInaccurate;
    case OsqpEigen::Status::DualInfeasible:
        return QpSolversEigen::Status::DualInfeasible;
    case OsqpEigen::Status::DualInfeasibleInaccurate:
        return QpSolversEigen::Status::DualInfeasibleInaccurate;
    case OsqpEigen::Status::MaxIterReached:
        return QpSolversEigen::Status::MaxIterReached;
    case OsqpEigen::Status::TimeLimitReached:
        return QpSolversEigen::Status::TimeLimitReached;
    case OsqpEigen::Status::NonCvx:
        return QpSolversEigen::Status::NonCvx;
    case OsqpEigen::Status::Sigint:
        return QpSolversEigen::Status::Sigint;
    case OsqpEigen::Status::Unsolved:
        return QpSolversEigen::Status::Unsolved;
    default:
        return QpSolversEigen::Status::SolverSpecificUnknownStatus;
    }
}

bool ProxqpSolver::convertQpSolversEigenInftyToOsqpEigenInfty(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& inputBound,
                                                            Eigen::VectorXd& outputBound)
{
    outputBound = inputBound;
    for (size_t i = 0; i < inputBound.size(); i++)
    {
        if (inputBound[i] == QpSolversEigen::INFTY)
        {
            outputBound[i] = OsqpEigen::INFTY;
        }

        if (inputBound[i] == -QpSolversEigen::INFTY)
        {
            outputBound[i] = -OsqpEigen::INFTY;
        }
    }
    return true;
}

std::string ProxqpSolver::getSolverName() const
{
    return "osqp";
}

bool ProxqpSolver::initSolver()
{
    return osqpEigenSolver.initSolver();
}

bool ProxqpSolver::isInitialized()
{
    return osqpEigenSolver.isInitialized();
}

void ProxqpSolver::clearSolver()
{
    return osqpEigenSolver.clearSolver();
}

bool ProxqpSolver::clearSolverVariables()
{
    return osqpEigenSolver.clearSolverVariables();
}

QpSolversEigen::ErrorExitFlag ProxqpSolver::solveProblem()
{
    return this->convertErrorExitFlag(osqpEigenSolver.solveProblem());
}

QpSolversEigen::Status ProxqpSolver::getStatus() const
{
    return this->convertStatus(osqpEigenSolver.getStatus());
}

const double ProxqpSolver::getObjValue() const
{
    return osqpEigenSolver.getObjValue();
}

const Eigen::Matrix<double, Eigen::Dynamic, 1>& ProxqpSolver::getSolution()
{
    return osqpEigenSolver.getSolution();
}

const Eigen::Matrix<double, Eigen::Dynamic, 1>& ProxqpSolver::getDualSolution()
{
    return osqpEigenSolver.getDualSolution();
}

bool ProxqpSolver::updateHessianMatrix(const Eigen::SparseMatrix<double> &hessianMatrix)
{
    return osqpEigenSolver.updateHessianMatrix(hessianMatrix);
}

bool ProxqpSolver::updateLinearConstraintsMatrix(const Eigen::SparseMatrix<double> &linearConstraintsMatrix)
{
    return osqpEigenSolver.updateLinearConstraintsMatrix(linearConstraintsMatrix);
}

bool ProxqpSolver::updateGradient(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& gradient)
{
    return osqpEigenSolver.updateGradient(gradient);
}

bool ProxqpSolver::updateLowerBound(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& lowerBound)
{
    bool ok = convertQpSolversEigenInftyToOsqpEigenInfty(lowerBound, lowerBoundBufferWithOsqpEigenInfty);
    return ok && osqpEigenSolver.updateLowerBound(lowerBoundBufferWithOsqpEigenInfty);
}

bool ProxqpSolver::updateUpperBound(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& upperBound)
{
    bool ok = convertQpSolversEigenInftyToOsqpEigenInfty(upperBound, upperBoundBufferWithOsqpEigenInfty);
    return ok && osqpEigenSolver.updateUpperBound(lowerBoundBufferWithOsqpEigenInfty);
}

bool ProxqpSolver::updateBounds(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& lowerBound,
             const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& upperBound)
{
    bool ok = convertQpSolversEigenInftyToOsqpEigenInfty(lowerBound, lowerBoundBufferWithOsqpEigenInfty);
    ok = ok &&  convertQpSolversEigenInftyToOsqpEigenInfty(upperBound, upperBoundBufferWithOsqpEigenInfty);
    return osqpEigenSolver.updateBounds(lowerBoundBufferWithOsqpEigenInfty, upperBoundBufferWithOsqpEigenInfty);
}

void ProxqpSolver::clearHessianMatrix()
{
    return osqpEigenSolver.data()->clearHessianMatrix();
}

void ProxqpSolver::clearLinearConstraintsMatrix()
{
    return osqpEigenSolver.data()->clearLinearConstraintsMatrix();
}

void ProxqpSolver::setNumberOfVariables(int n)
{
    return osqpEigenSolver.data()->setNumberOfVariables(n);
}

void ProxqpSolver::setNumberOfConstraints(int m)
{
    return osqpEigenSolver.data()->setNumberOfConstraints(m);
}

bool ProxqpSolver::setHessianMatrix(const Eigen::SparseMatrix<double>& hessianMatrix)
{
    return osqpEigenSolver.data()->setHessianMatrix(hessianMatrix);
}

bool ProxqpSolver::setGradient(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> gradientVector)
{
    return  osqpEigenSolver.data()->setGradient(gradientVector);
}

Eigen::Matrix<double, Eigen::Dynamic, 1> ProxqpSolver::getGradient()
{
    return Eigen::Matrix<double, Eigen::Dynamic, 1>();
}

bool
ProxqpSolver::setLinearConstraintsMatrix(const Eigen::SparseMatrix<double>& linearConstraintsMatrix)
{
    return osqpEigenSolver.data()->setLinearConstraintsMatrix(linearConstraintsMatrix);
}

bool ProxqpSolver::setLowerBound(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> lowerBoundVector)
{
    bool ok = convertQpSolversEigenInftyToOsqpEigenInfty(lowerBoundVector, lowerBoundBufferWithOsqpEigenInfty);
    return osqpEigenSolver.data()->setLowerBound(lowerBoundBufferWithOsqpEigenInfty);
}

bool ProxqpSolver::setUpperBound(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> upperBoundVector)
{
    bool ok = convertQpSolversEigenInftyToOsqpEigenInfty(upperBoundVector, upperBoundBufferWithOsqpEigenInfty);
    return osqpEigenSolver.data()->setUpperBound(upperBoundVector);
}

bool ProxqpSolver::setBounds(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> lowerBound,
               Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> upperBound)
{
    bool ok = convertQpSolversEigenInftyToOsqpEigenInfty(lowerBound, lowerBoundBufferWithOsqpEigenInfty);
    ok = ok &&  convertQpSolversEigenInftyToOsqpEigenInfty(upperBound, upperBoundBufferWithOsqpEigenInfty);
    return osqpEigenSolver.data()->setBounds(lowerBoundBufferWithOsqpEigenInfty, upperBoundBufferWithOsqpEigenInfty);
}

bool ProxqpSolver::setBooleanParameter(const std::string& settingName, bool value)
{
    // If you edit this method, remember to update
    // the documentation in the README of the osqp plugin
    if (settingName == "polish")
    {
        osqpEigenSolver.settings()->setPolish(value);
        return true;
    } else if (settingName == "verbose")
    {
        osqpEigenSolver.settings()->setVerbosity(value);
        return true;
    } else if (settingName == "scaled_termination")
    {
        osqpEigenSolver.settings()->setScaledTerimination(value);
        return true;
    } else if (settingName == "warm_start")
    {
        osqpEigenSolver.settings()->setWarmStart(value);
        return true;
    } else if (settingName == "warm_start")
    {
        osqpEigenSolver.settings()->setWarmStart(value);
        return true;
    } else if (settingName == "adaptive_rho")
    {
        osqpEigenSolver.settings()->setAdaptiveRho(value);
        return true;
    }

    QpSolversEigen::debugStream() << "QpSolversEigen::ProxqpSolver::setBooleanParameter: unknown setting name: " << settingName << std::endl;
    return false;
}

bool ProxqpSolver::setIntegerParameter(const std::string& settingName, int64_t value)
{
    // If you edit this method, remember to update
    // the documentation in the README of the osqp plugin
    if (settingName == "scaling")
    {
        osqpEigenSolver.settings()->setScaling(value);
        return true;
    } else if (settingName == "adaptive_rho_interval")
    {
        osqpEigenSolver.settings()->setAdaptiveRhoInterval(value);
        return true;
    } else if (settingName == "max_iter")
    {
        osqpEigenSolver.settings()->setMaxIteration(value);
        return true;
    } else if (settingName == "polish_refine_iter")
    {
        osqpEigenSolver.settings()->setPolishRefineIter(value);
        return true;
    } else if (settingName == "linsys_solver")
    {
        osqpEigenSolver.settings()->setLinearSystemSolver(value);
        return true;
    } else if (settingName == "check_termination")
    {
        osqpEigenSolver.settings()->setCheckTermination(value);
        return true;
    }

    QpSolversEigen::debugStream() << "QpSolversEigen::ProxqpSolver::setIntegerParameter: unknown setting name: " << settingName << std::endl;
    return false;
}

bool ProxqpSolver::setRealNumberParameter(const std::string& settingName, double value)
{
    // If you edit this method, remember to update
    // the documentation in the README of the osqp plugin
    if (settingName == "rho")
    {
        osqpEigenSolver.settings()->setRho(value);
        return true;
    } else if (settingName == "sigma")
    {
        osqpEigenSolver.settings()->setSigma(value);
        return true;
    } else if (settingName == "adaptive_rho_tolerance")
    {
        osqpEigenSolver.settings()->setAdaptiveRhoTolerance(value);
        return true;
    } else if (settingName == "adaptive_rho_fraction")
    {
        osqpEigenSolver.settings()->setAdaptiveRhoFraction(value);
        return true;
    } else if (settingName == "eps_abs")
    {
        osqpEigenSolver.settings()->setAbsoluteTolerance(value);
        return true;
    } else if (settingName == "eps_rel")
    {
        osqpEigenSolver.settings()->setRelativeTolerance(value);
        return true;
    } else if (settingName == "eps_prim_inf")
    {
        osqpEigenSolver.settings()->setPrimalInfeasibilityTolerance(value);
        return true;
    } else if (settingName == "eps_dual_inf")
    {
        osqpEigenSolver.settings()->setDualInfeasibilityTolerance(value);
        return true;
    } else if (settingName == "alpha")
    {
        osqpEigenSolver.settings()->setAlpha(value);
        return true;
    } else if (settingName == "delta")
    {
        osqpEigenSolver.settings()->setDelta(value);
        return true;
    }

    QpSolversEigen::debugStream() << "QpSolversEigen::ProxqpSolver::setRealNumberParameter: unknown setting name: " << settingName << std::endl;
    return false;
}

SolverInterface* ProxqpSolver::allocateInstance() const
{
    return new ProxqpSolver();
}

}