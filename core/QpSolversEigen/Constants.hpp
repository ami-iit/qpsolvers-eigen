#ifndef QPSOLVERSEIGEN_CONSTANTS_HPP
#define QPSOLVERSEIGEN_CONSTANTS_HPP

#include <limits>
#include <string>

namespace QpSolversEigen
{
constexpr double INFTY = std::numeric_limits<double>::infinity(); /**< Infinity constant for QpSolversEigen, will be translated to solver specific constants if necessary */

/**
 * Status of the solver.
 *
 * For historical reasons, the status code resemble the one of OSQP, however this is not something that 
 * is guaranteed to be true in the future. Each solver status will be mapped to this QpSolversEigen specific
 * status code. If the solver status of the solver does not match any of the listed status, the status reported
 * will be SolverSpecificUnknownStatus.
 */
enum class Status : int
{
    Solved = 1,
    SolvedInaccurate = 2,
    PrimalInfeasible = 3,
    PrimalInfeasibleInaccurate = 4,
    DualInfeasible = 5,
    DualInfeasibleInaccurate = 6,
    MaxIterReached = 7,
    TimeLimitReached = 8,
    NonCvx = 9,
    Sigint = 10,
    Unsolved = 11,
    SolverNotInitialized = 12,
    SolvedClosestPrimalFeasible = 13,
    SolverSpecificUnknownStatus = 1000,
};

/**
 * Error exit flag of the Solver
 * 
 * For historical reasons, the error exit flag code resemble the one of OSQP, however this is not something that 
 * is guaranteed to be true in the future. Each solver exit flag will be mapped to this QpSolversEigen specific
 * ErrorExitFlag code. If the exit code status of the solver does not match any of the listed status, the status reported
 * will be SolverSpecificUnknownError.
 */
enum class ErrorExitFlag : int
{
    NoError = 0,
    DataValidationError = 1,
    SettingsValidationError = 2,
    LinsysSolverLoadError = 3,
    LinsysSolverInitError = 4,
    NonCvxError = 5,
    MemAllocError = 6,
    WorkspaceNotInitError = 7,
    SolverSpecificUnknownError = 1000,
};

/**
 * Get the library name from the solver name
 */
inline std::string getSharedlibppLibraryNameFromSolverName(const std::string& solverName)
{
    return "qpsolvers-eigen-" + solverName;
}

/**
 * Get the factory name from the solver name
 */
inline std::string getSharedlibppFactoryNameFromSolverName(const std::string& solverName)
{
    return "qpsolvers_eigen_" + solverName;
}

}

#endif
