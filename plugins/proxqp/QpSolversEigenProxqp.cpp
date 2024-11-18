// QpSolversEigen 
#include <QpSolversEigen/SolverInterface.hpp>
#include <QpSolversEigen/Debug.hpp>

// Class factory API
#include <sharedlibpp/SharedLibraryClassApi.h>

// proxsuite
#include <proxsuite/proxqp/sparse/sparse.hpp>

#include <memory>

namespace QpSolversEigen
{

class ProxqpSolver final: public SolverInterface
{
private:
    // proxqp settings
    proxsuite::proxqp::Settings<double> proxqpSettings;
    // proxqp sparse solver
    std::unique_ptr<proxsuite::proxqp::sparse::QP<double, long long>> proxqpSparseSolver;

    struct InitialSparseSolverData {
        int numberOfVariables = 0;
        int numberOfConstraints = 0;

        // Hessian
        bool isHessianSet = false;
        Eigen::SparseMatrix<double> H;

        // Gradient
        bool isGradientSet = false;
        Eigen::VectorXd g;

        // Equality constraints data (unused)
        Eigen::SparseMatrix<double> A;
        Eigen::VectorXd b;

        // Inequality constraints data
        bool isLinearConstraintsSet = false;
        Eigen::SparseMatrix<double> C;
        bool isLowerBoundSet = false;
        Eigen::VectorXd l;
        bool isUpperBoundSet = false;
        Eigen::VectorXd u;


        bool isSet()
        {
            return isHessianSet && isGradientSet &&
                   // If the problem is unconstrained there is no need to set the constraint-related variables
                   ((isLinearConstraintsSet && isLowerBoundSet && isUpperBoundSet) || (numberOfConstraints == 0));
        }
    } initialSparseSolverData;



    // If proxqpSparseSolver is set, copy proxqpSettings to the proxqpSparseSolver->settings
    void syncSettings();

    // Helper class to convert proxsuite::proxqp::QPSolverOutput to QpSolversEigen::Status
    QpSolversEigen::Status convertStatus(const proxsuite::proxqp::QPSolverOutput status) const;

public:
    virtual ~ProxqpSolver() = default;

    std::string getSolverName() const override;
    void setNumberOfVariables(int n) override;
    void setNumberOfConstraints(int m) override;
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
    bool setStringParameter(const std::string& parameterName, const std::string& value) override;

    bool getBooleanParametersNames(std::vector<std::string>& parametersNames) const override;
    bool getIntegerParametersNames(std::vector<std::string>& parametersNames) const override;
    bool getRealNumberParametersNames(std::vector<std::string>& parametersNames) const override;
    bool getStringParametersNames(std::vector<std::string>& parametersNames) const override;

    SolverInterface* allocateInstance() const override;
};

// The first argument needs to be coherent with the scheme used in
// getSharedlibppFactoryNameFromSolverName, i.e. qpsolvers_eigen_<solverName>
SHLIBPP_DEFINE_SHARED_SUBCLASS(qpsolvers_eigen_proxqp, QpSolversEigen::ProxqpSolver, QpSolversEigen::SolverInterface);

QpSolversEigen::Status ProxqpSolver::convertStatus(const proxsuite::proxqp::QPSolverOutput status) const
{
    switch (status)
    {
    case proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED:
        return QpSolversEigen::Status::Solved;
    case proxsuite::proxqp::QPSolverOutput::PROXQP_MAX_ITER_REACHED:
        return QpSolversEigen::Status::MaxIterReached;
    case proxsuite::proxqp::QPSolverOutput::PROXQP_PRIMAL_INFEASIBLE:
        return QpSolversEigen::Status::PrimalInfeasible;
    case proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED_CLOSEST_PRIMAL_FEASIBLE:
        return QpSolversEigen::Status::SolvedClosestPrimalFeasible;
    case proxsuite::proxqp::QPSolverOutput::PROXQP_DUAL_INFEASIBLE:
        return QpSolversEigen::Status::DualInfeasible;
    case proxsuite::proxqp::QPSolverOutput::PROXQP_NOT_RUN:
        return QpSolversEigen::Status::Unsolved;
    default:
        return QpSolversEigen::Status::SolverSpecificUnknownStatus;
    }
}

void ProxqpSolver::syncSettings()
{
    if (proxqpSparseSolver)
    {
        proxqpSparseSolver->settings = proxqpSettings;
    }
}

std::string ProxqpSolver::getSolverName() const
{
    return "proxqp";
}

void ProxqpSolver::setNumberOfVariables(int n)
{
    initialSparseSolverData.numberOfVariables = n;
}

void ProxqpSolver::setNumberOfConstraints(int m)
{
    initialSparseSolverData.numberOfConstraints = m;
}

bool ProxqpSolver::initSolver()
{
    if (!initialSparseSolverData.isSet())
    {
        debugStream() << "QpSolversEigen::ProxqpSolver::initSolver: Some data are not set." << std::endl;
        return false;
    }

    // See https://github.com/ami-iit/qpsolvers-eigen/issues/4
    int numberOfEqualityConstraints = 0;
    proxqpSparseSolver = std::make_unique<proxsuite::proxqp::sparse::QP<double, long long>>(initialSparseSolverData.numberOfVariables, numberOfEqualityConstraints, initialSparseSolverData.numberOfConstraints);
    syncSettings();
    proxqpSparseSolver->init(initialSparseSolverData.H, initialSparseSolverData.g,
                             initialSparseSolverData.A, initialSparseSolverData.b,
                             initialSparseSolverData.C, initialSparseSolverData.l, initialSparseSolverData.u);

    return isInitialized();
}

bool ProxqpSolver::isInitialized()
{
    return static_cast<bool>(proxqpSparseSolver);
}

void ProxqpSolver::clearSolver()
{
    proxqpSparseSolver.reset();
}

bool ProxqpSolver::clearSolverVariables()
{
    QpSolversEigen::debugStream() << "QpSolversEigen::ProxqpSolver::clearSolverVariables: method not supported in proxqp." << std::endl;
    return false;
}

QpSolversEigen::ErrorExitFlag ProxqpSolver::solveProblem()
{
    if (!proxqpSparseSolver)
    {
        QpSolversEigen::debugStream() << "QpSolversEigen::ProxqpSolver::solveProblem: solver not initialized." << std::endl;
        return QpSolversEigen::ErrorExitFlag::WorkspaceNotInitError;
    }

    proxqpSparseSolver->solve();


    return QpSolversEigen::ErrorExitFlag::NoError;
}

QpSolversEigen::Status ProxqpSolver::getStatus() const
{
    if (!proxqpSparseSolver)
    {
        QpSolversEigen::debugStream() << "QpSolversEigen::ProxqpSolver::solveProblem: solver not initialized." << std::endl;
        return QpSolversEigen::Status::SolverNotInitialized;
    }

    return this->convertStatus(proxqpSparseSolver->results.info.status);
}

const double ProxqpSolver::getObjValue() const
{
    return proxqpSparseSolver->results.info.objValue;
}

const Eigen::Matrix<double, Eigen::Dynamic, 1>& ProxqpSolver::getSolution()
{
    return proxqpSparseSolver->results.x;
}

const Eigen::Matrix<double, Eigen::Dynamic, 1>& ProxqpSolver::getDualSolution()
{
    return proxqpSparseSolver->results.z;
}

bool ProxqpSolver::updateHessianMatrix(const Eigen::SparseMatrix<double> &hessianMatrix)
{
    if (!isInitialized())
    {
        debugStream() << "QpSolversEigen::ProxqpSolver::updateHessianMatrix: solver was not initialized." << std::endl;
        return false;
    }

    proxqpSparseSolver->update(initialSparseSolverData.H, proxsuite::nullopt,
                               proxsuite::nullopt, proxsuite::nullopt,
                               proxsuite::nullopt, proxsuite::nullopt, proxsuite::nullopt);

    return true;
}

bool ProxqpSolver::updateLinearConstraintsMatrix(const Eigen::SparseMatrix<double> &linearConstraintsMatrix)
{
    proxqpSparseSolver->update(proxsuite::nullopt, proxsuite::nullopt,
                               proxsuite::nullopt, proxsuite::nullopt,
                               linearConstraintsMatrix, proxsuite::nullopt, proxsuite::nullopt);
    return true;
}

bool ProxqpSolver::updateGradient(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& gradient)
{
    proxqpSparseSolver->update(proxsuite::nullopt, gradient,
                               proxsuite::nullopt, proxsuite::nullopt,
                               proxsuite::nullopt, proxsuite::nullopt, proxsuite::nullopt);
    return true;
}

bool ProxqpSolver::updateLowerBound(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& lowerBound)
{
    proxqpSparseSolver->update(proxsuite::nullopt, proxsuite::nullopt,
                               proxsuite::nullopt, proxsuite::nullopt,
                               proxsuite::nullopt, lowerBound, proxsuite::nullopt);
    return true;
}

bool ProxqpSolver::updateUpperBound(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& upperBound)
{
    proxqpSparseSolver->update(proxsuite::nullopt, proxsuite::nullopt,
                               proxsuite::nullopt, proxsuite::nullopt,
                               proxsuite::nullopt, proxsuite::nullopt, upperBound);
    return true;
}

bool ProxqpSolver::updateBounds(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& lowerBound,
             const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& upperBound)
{
    proxqpSparseSolver->update(proxsuite::nullopt, proxsuite::nullopt,
                               proxsuite::nullopt, proxsuite::nullopt,
                               proxsuite::nullopt, lowerBound, upperBound);
    return true;
}

void ProxqpSolver::clearHessianMatrix()
{
    QpSolversEigen::debugStream() << "QpSolversEigen::ProxqpSolver::clearHessianMatrix: method unsupported in proxqp plugin" << std::endl;
    return;
}

void ProxqpSolver::clearLinearConstraintsMatrix()
{
    QpSolversEigen::debugStream() << "QpSolversEigen::ProxqpSolver::clearLinearConstraintsMatrix: method unsupported in proxqp plugin" << std::endl;
    return;
}

bool ProxqpSolver::setHessianMatrix(const Eigen::SparseMatrix<double>& hessianMatrix)
{
    initialSparseSolverData.H = hessianMatrix;
    initialSparseSolverData.isHessianSet = true;
    return true;
}

bool ProxqpSolver::setGradient(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> gradientVector)
{
    initialSparseSolverData.g = gradientVector;
    initialSparseSolverData.isGradientSet = true;
    return true;
}

Eigen::Matrix<double, Eigen::Dynamic, 1> ProxqpSolver::getGradient()
{
    return Eigen::Matrix<double, Eigen::Dynamic, 1>();
}

bool
ProxqpSolver::setLinearConstraintsMatrix(const Eigen::SparseMatrix<double>& linearConstraintsMatrix)
{
    initialSparseSolverData.C = linearConstraintsMatrix;
    initialSparseSolverData.isLinearConstraintsSet = true;
    return true;
}

bool ProxqpSolver::setLowerBound(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> lowerBoundVector)
{
    initialSparseSolverData.l = lowerBoundVector;
    initialSparseSolverData.isLowerBoundSet = true;
    return true;
}

bool ProxqpSolver::setUpperBound(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> upperBoundVector)
{
    initialSparseSolverData.u = upperBoundVector;
    initialSparseSolverData.isUpperBoundSet = true;
    return true;
}

bool ProxqpSolver::setBounds(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> lowerBound,
               Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> upperBound)
{
    bool ok = setLowerBound(lowerBound) && setUpperBound(upperBound);
    return ok;
}

bool ProxqpSolver::setBooleanParameter(const std::string& settingName, bool value)
{
    // If you edit this method, remember to update
    // the documentation in the README of the proxqp plugin
    bool settingFound = false;
    if (settingName == "verbose")
    {
        proxqpSettings.verbose = value;
        settingFound = true;
    } else if (settingName == "update_preconditioner")
    {
        proxqpSettings.update_preconditioner = value;
        settingFound = true;
    } else if (settingName == "compute_preconditioner")
    {
        proxqpSettings.compute_preconditioner = value;
        settingFound = true;
    } else if (settingName == "compute_timings")
    {
        proxqpSettings.compute_timings = value;
        settingFound = true;
    } else if (settingName == "check_duality_gap")
    {
        proxqpSettings.check_duality_gap = value;
        settingFound = true;
    } else if (settingName == "bcl_update")
    {
        proxqpSettings.bcl_update = value;
        settingFound = true;
    } else if (settingName == "primal_infeasibility_solving")
    {
        proxqpSettings.primal_infeasibility_solving = value;
        settingFound = true;
    }

    if (settingFound)
    {
        syncSettings();
        return true;
    }

    QpSolversEigen::debugStream() << "QpSolversEigen::ProxqpSolver::setRealNumberParameter: unknown setting name: " << settingName << std::endl;
    return false;
}

bool ProxqpSolver::setIntegerParameter(const std::string& settingName, int64_t value)
{
    bool settingFound = false;
    if (settingName == "max_iter")
    {
        proxqpSettings.max_iter = value;
        settingFound = true;
    } else if (settingName == "max_iter_in")
    {
        proxqpSettings.max_iter_in = value;
        settingFound = true;
    } else if (settingName == "safe_guard")
    {
        proxqpSettings.safe_guard = value;
        settingFound = true;
    } else if (settingName == "nb_iterative_refinement")
    {
        proxqpSettings.nb_iterative_refinement = value;
        settingFound = true;
    } else if (settingName == "preconditioner_max_iter")
    {
        proxqpSettings.preconditioner_max_iter = value;
        settingFound = true;
    } else if (settingName == "frequence_infeasibility_check")
    {
        proxqpSettings.frequence_infeasibility_check = value;
        settingFound = true;
    }

    if (settingFound)
    {
        syncSettings();
        return true;
    }

    QpSolversEigen::debugStream() << "QpSolversEigen::ProxqpSolver::setRealNumberParameter: unknown setting name: " << settingName << std::endl;
    return false;
}

bool ProxqpSolver::setRealNumberParameter(const std::string& settingName, double value)
{
    // If you edit this method, remember to update
    // the documentation in the README of the osqp plugin
    bool settingFound = false;
    if (settingName == "default_mu_eq")
    {
        proxqpSettings.default_mu_eq = value;
        settingFound = true;
    } else if (settingName == "default_mu_in")
    {
        proxqpSettings.default_mu_in = value;
        settingFound = true;
    } else if (settingName == "alpha_bcl")
    {
        proxqpSettings.alpha_bcl = value;
        settingFound = true;
    } else if (settingName == "beta_bcl")
    {
        proxqpSettings.beta_bcl = value;
        settingFound = true;
    } else if (settingName == "refactor_dual_feasibility_threshold")
    {
        proxqpSettings.refactor_dual_feasibility_threshold = value;
        settingFound = true;
    } else if (settingName == "refactor_rho_threshold")
    {
        proxqpSettings.refactor_rho_threshold = value;
        settingFound = true;
    } else if (settingName == "mu_min_in")
    {
        proxqpSettings.mu_min_in = value;
        settingFound = true;
    } else if (settingName == "mu_max_eq_inv")
    {
        proxqpSettings.mu_max_eq_inv = value;
        settingFound = true;
    } else if (settingName == "mu_max_in_inv")
    {
        proxqpSettings.mu_max_in_inv = value;
        settingFound = true;
    } else if (settingName == "mu_update_factor")
    {
        proxqpSettings.mu_update_factor = value;
        settingFound = true;
    } else if (settingName == "cold_reset_mu_eq")
    {
        proxqpSettings.cold_reset_mu_eq = value;
        settingFound = true;
    } else if (settingName == "cold_reset_mu_in")
    {
        proxqpSettings.cold_reset_mu_in = value;
        settingFound = true;
    } else if (settingName == "cold_reset_mu_eq_inv")
    {
        proxqpSettings.cold_reset_mu_eq_inv = value;
        settingFound = true;
    } else if (settingName == "cold_reset_mu_in_inv")
    {
        proxqpSettings.cold_reset_mu_in_inv = value;
        settingFound = true;
    } else if (settingName == "eps_abs")
    {
        proxqpSettings.eps_abs = value;
        settingFound = true;
    } else if (settingName == "eps_rel")
    {
        proxqpSettings.eps_rel = value;
        settingFound = true;
    } else if (settingName == "eps_refact")
    {
        proxqpSettings.eps_refact = value;
        settingFound = true;
    } else if (settingName == "eps_duality_gap_abs")
    {
        proxqpSettings.eps_duality_gap_abs = value;
        settingFound = true;
    } else if (settingName == "eps_duality_gap_rel")
    {
        proxqpSettings.eps_duality_gap_rel = value;
        settingFound = true;
    } else if (settingName == "preconditioner_accuracy")
    {
        proxqpSettings.preconditioner_accuracy = value;
        settingFound = true;
    } else if (settingName == "alpha_gpdal")
    {
        proxqpSettings.alpha_gpdal = value;
        settingFound = true;
    } else if (settingName == "default_H_eigenvalue_estimate")
    {
        proxqpSettings.default_H_eigenvalue_estimate = value;
        settingFound = true;
    }

    if (settingFound)
    {
        syncSettings();
        return true;
    }

    QpSolversEigen::debugStream() << "QpSolversEigen::ProxqpSolver::setRealNumberParameter: unknown setting name: " << settingName << std::endl;
    return false;
}

bool ProxqpSolver::setStringParameter(const std::string& parameterName, const std::string& value)
{
    bool settingFound = false;
    bool valueFound = false;

    if (parameterName == "initial_guess")
    {
        settingFound = true;
        if (value == "NO_INITIAL_GUESS")
        {
            proxqpSettings.initial_guess = proxsuite::proxqp::InitialGuessStatus::NO_INITIAL_GUESS;
            valueFound = true;
        } else if (value == "EQUALITY_CONSTRAINED_INITIAL_GUESS")
        {
            proxqpSettings.initial_guess = proxsuite::proxqp::InitialGuessStatus::EQUALITY_CONSTRAINED_INITIAL_GUESS;
            valueFound = true;
        } else if (value == "WARM_START_WITH_PREVIOUS_RESULT")
        {
            proxqpSettings.initial_guess = proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
            valueFound = true;
        } else if (value == "WARM_START")
        {
            proxqpSettings.initial_guess = proxsuite::proxqp::InitialGuessStatus::WARM_START;
            valueFound = true;
        } else if (value == "COLD_START_WITH_PREVIOUS_RESULT")
        {
            proxqpSettings.initial_guess = proxsuite::proxqp::InitialGuessStatus::COLD_START_WITH_PREVIOUS_RESULT;
            valueFound = true;
        }
    }


    if (settingFound && valueFound)
    {
        syncSettings();
        return true;
    }

    if (!settingFound)
    {
        QpSolversEigen::debugStream() << "QpSolversEigen::ProxqpSolver::setStringParameter: unknown setting name: " << parameterName << std::endl;
    } else if (!valueFound)
    {
        QpSolversEigen::debugStream() << "QpSolversEigen::ProxqpSolver::setStringParameter: unknown value << " << value << " for parameter with name: " << parameterName << std::endl;
    }
    return false;
}

bool ProxqpSolver::getBooleanParametersNames(std::vector<std::string>& parametersNames) const
{
    parametersNames.clear();
    parametersNames.push_back("verbose");
    parametersNames.push_back("update_preconditioner");
    parametersNames.push_back("compute_preconditioner");
    parametersNames.push_back("compute_timings");
    parametersNames.push_back("check_duality_gap");
    parametersNames.push_back("bcl_update");
    parametersNames.push_back("primal_infeasibility_solving");
    return true;
}

bool ProxqpSolver::getIntegerParametersNames(std::vector<std::string>& parametersNames) const
{
    parametersNames.clear();
    parametersNames.push_back("max_iter");
    parametersNames.push_back("max_iter_in");
    parametersNames.push_back("safe_guard");
    parametersNames.push_back("nb_iterative_refinement");
    parametersNames.push_back("preconditioner_max_iter");
    parametersNames.push_back("frequence_infeasibility_check");
    return true;
}

bool ProxqpSolver::getRealNumberParametersNames(std::vector<std::string>& parametersNames) const
{
    parametersNames.clear();
    parametersNames.push_back("default_mu_eq");
    parametersNames.push_back("default_mu_in");
    parametersNames.push_back("alpha_bcl");
    parametersNames.push_back("beta_bcl");
    parametersNames.push_back("refactor_dual_feasibility_threshold");
    parametersNames.push_back("refactor_rho_threshold");
    parametersNames.push_back("mu_min_in");
    parametersNames.push_back("mu_max_eq_inv");
    parametersNames.push_back("mu_max_in_inv");
    parametersNames.push_back("mu_update_factor");
    parametersNames.push_back("cold_reset_mu_eq");
    parametersNames.push_back("cold_reset_mu_in");
    parametersNames.push_back("cold_reset_mu_eq_inv");
    parametersNames.push_back("cold_reset_mu_in_inv");
    parametersNames.push_back("eps_abs");
    parametersNames.push_back("eps_rel");
    parametersNames.push_back("eps_refact");
    parametersNames.push_back("eps_duality_gap_abs");
    parametersNames.push_back("eps_duality_gap_rel");
    parametersNames.push_back("preconditioner_accuracy");
    parametersNames.push_back("alpha_gpdal");
    parametersNames.push_back("default_H_eigenvalue_estimate");
    return true;
}

bool ProxqpSolver::getStringParametersNames(std::vector<std::string>& parametersNames) const
{
    parametersNames.clear();
    parametersNames.push_back("initial_guess");
    return true;
}

SolverInterface* ProxqpSolver::allocateInstance() const
{
    return new ProxqpSolver();
}

}