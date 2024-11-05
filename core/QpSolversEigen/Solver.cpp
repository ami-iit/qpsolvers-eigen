#include <QpSolversEigen/Solver.hpp>
#include <QpSolversEigen/SolverInterface.hpp>
#include <QpSolversEigen/NullSolver.hpp>
#include <QpSolversEigen/Debug.hpp>

#include <sharedlibpp/SharedLibraryClassFactory.h>
#include <sharedlibpp/SharedLibraryClass.h>

namespace QpSolversEigen
{

struct Solver::Impl
{
    bool solverInitialized;
    std::unique_ptr<QpSolversEigen::SolverInterface> solver;
    std::unique_ptr<sharedlibpp::SharedLibraryClassFactory<QpSolversEigen::SolverInterface>> solverFactory;
    Impl();
};

Solver::Impl::Impl()
    : solverInitialized{false},
      solver{std::make_unique<QpSolversEigen::NullSolver>()}
{
}

Solver::Solver()
    : m_pimpl{std::make_unique<Impl>()}
{
}

Solver::~Solver()
{
    // Ensure that the solver is deallocated before the factory
    m_pimpl->solver.reset();
    m_pimpl->solverFactory.reset();
}

bool Solver::instantiateSolver(std::string solverName)
{
    // Ensure that the solver is deallocated before the factory
    m_pimpl->solver.reset();

    // Get the expected library name and factory name given the solver name
    std::string libraryName = getSharedlibppLibraryNameFromSolverName(solverName);
    std::string factoryName = getSharedlibppFactoryNameFromSolverName(solverName);
    m_pimpl->solverFactory = std::make_unique<sharedlibpp::SharedLibraryClassFactory<QpSolversEigen::SolverInterface>>(libraryName.c_str(), factoryName.c_str());

    if (!m_pimpl->solverFactory->isValid())
    {
        debugStream() << "QpSolversEigen::Solver::initSolver: Failed to create factory for solver " << solverName << std::endl;
        debugStream() << "Factory error (" << static_cast<std::uint32_t>(m_pimpl->solverFactory->getStatus())
                    << "): " << m_pimpl->solverFactory->getError().c_str() << std::endl;
        debugStream() << "Make sure that the library name contains " << libraryName << " and the factory name is called " << factoryName << std::endl;
        return false;
    }

    sharedlibpp::SharedLibraryClass<QpSolversEigen::SolverInterface> solver(*(m_pimpl->solverFactory));
    m_pimpl->solver.reset(solver.getContent().allocateInstance());
    return true;
}

bool Solver::initSolver()
{
    return m_pimpl->solver->initSolver();
}


bool Solver::isInitialized()
{
    return m_pimpl->solver->isInitialized();
}

void Solver::clearSolver()
{
    m_pimpl->solver->clearSolver();
}

bool Solver::clearSolverVariables()
{
    return m_pimpl->solver->clearSolverVariables();
}

QpSolversEigen::ErrorExitFlag Solver::solveProblem()
{
    return m_pimpl->solver->solveProblem();
}

QpSolversEigen::Status Solver::getStatus() const
{
    return m_pimpl->solver->getStatus();
}

const double Solver::getObjValue() const
{
    return m_pimpl->solver->getObjValue();
}

const Eigen::Matrix<double, -1, 1>& Solver::getSolution()
{
    return m_pimpl->solver->getSolution();
}

const Eigen::Matrix<double, -1, 1>& Solver::getDualSolution()
{
    return m_pimpl->solver->getDualSolution();
}

bool Solver::updateHessianMatrix(const Eigen::SparseMatrix<double> &hessianMatrix)
{
    return m_pimpl->solver->updateHessianMatrix(hessianMatrix);
}

bool Solver::updateLinearConstraintsMatrix(const Eigen::SparseMatrix<double> &linearConstraintsMatrix)
{
    return m_pimpl->solver->updateLinearConstraintsMatrix(linearConstraintsMatrix);
}

bool
Solver::updateGradient(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& gradient)
{
    return m_pimpl->solver->updateGradient(gradient);
}

bool
Solver::updateLowerBound(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& lowerBound)
{
    return m_pimpl->solver->updateLowerBound(lowerBound);
}

bool
Solver::updateUpperBound(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& upperBound)
{
    return m_pimpl->solver->updateUpperBound(upperBound);
}

bool
Solver::updateBounds(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& lowerBound,
             const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& upperBound)
{
    return m_pimpl->solver->updateBounds(lowerBound, upperBound);
}

void Solver::clearHessianMatrix()
{
    return m_pimpl->solver->clearHessianMatrix();
}

void Solver::clearLinearConstraintsMatrix()
{
    return m_pimpl->solver->clearLinearConstraintsMatrix();
}

void Solver::setNumberOfVariables(int n)
{
    return m_pimpl->solver->setNumberOfVariables(n);
}

void Solver::setNumberOfConstraints(int m)
{
    return m_pimpl->solver->setNumberOfConstraints(m);
}

bool Solver::setHessianMatrix(const Eigen::SparseMatrix<double>& hessianMatrix)
{
    return m_pimpl->solver->setHessianMatrix(hessianMatrix);
}

bool Solver::setGradient(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> gradientVector)
{
    return m_pimpl->solver->setGradient(gradientVector);
}

Eigen::Matrix<double, Eigen::Dynamic, 1> Solver::getGradient()
{
    return m_pimpl->solver->getGradient();
}

bool
Solver::setLinearConstraintsMatrix(const Eigen::SparseMatrix<double>& linearConstraintsMatrix)
{
    return m_pimpl->solver->setLinearConstraintsMatrix(linearConstraintsMatrix);
}

bool Solver::setLowerBound(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> lowerBoundVector)
{
    return m_pimpl->solver->setLowerBound(lowerBoundVector);
}

bool Solver::setUpperBound(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> upperBoundVector)
{
    return m_pimpl->solver->setUpperBound(upperBoundVector);
}

bool Solver::setBounds(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> lowerBound,
               Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> upperBound)
{
    return m_pimpl->solver->setBounds(lowerBound, upperBound);
}

bool Solver::setBooleanParameter(const std::string& parameterName, bool value)
{
    return m_pimpl->solver->setBooleanParameter(parameterName, value);
}

bool Solver::setIntegerParameter(const std::string& parameterName, int64_t value)
{
    return m_pimpl->solver->setIntegerParameter(parameterName, value);
}

bool Solver::setRealNumberParameter(const std::string& parameterName, double value)
{
    return m_pimpl->solver->setRealNumberParameter(parameterName, value);
}

Solver* Solver::data()
{
    return this;
}

}

