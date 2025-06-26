# How to migrate from osqp-eigen to qpsolvers-eigen

This document list the changes that you need to do to migrate from `osqp-eigen` to `qpsolvers-eigen`.

## CMake

Change any instance of `find_package(OsqpEigen)` to `find_package(QpSolversEigen)` and any instance of `OsqpEigen::OsqpEigen` to `QpSolversEigen::QpSolversEigen`.

## Header inclusion

Remove any line that include OsqpEigen headers:

~~~cxx
#include <OsqpEigen/...>
~~~

and instead include:

~~~cxx
#include <QpSolversEigen/QpSolversEigen.hpp>
~~~

## Solver instantiation

With osqp-eigen, the solver was instantiated as:

~~~cxx
OsqpEigen::Solver solver;
~~~

while for qpsolvers-eigen you need to instantiate the solver with:

~~~cxx
QpSolversEigen::Solver solver;
// Change "osqp" to any solver you want to use
solver.instantiateSolver("osqp");
~~~

## solver.data() methods

All calls to methods like:

~~~cxx
solver.data()->setHessianMatrix(hessian)
solver.data()->setGradient(gradient)
~~~

Need to be changed by removing `data()->`, i.e. converted to:

~~~cxx
solver.setHessianMatrix(hessian)
solver.setGradient(gradient)
~~~

One difference between osqp-eigen and qpsolvers-eigen is that qpsolvers-eigen support explicit equality constraints, so the method related to set inequality constraints have new names:

| `OsqpEigen` name | `QpSolversEigen` name |
|:----------------:|:----------------------:|
| `updateLinearConstraintsMatrix()` | `updateInequalityConstraintsMatrix()`  |
| `setLinearConstraintsMatrix()`    | `setInequalityConstraintsMatrix()`     |
| `setNumberOfConstraints()`        | `setNumberOfInequalityConstraints()`   |

If in your osqp-eigen code you were converting equality constraints to inequality constraints, you can now directly set your equality constraints.

## solver.settings() methods

As `QpSolversEigen` has a generic solver-agnostic interface, to set parameters you need to specify the parameter to set by name, so you need to port code such as:

~~~cxx
solver.settings()->setVerbosity(true);
solver.settings()->setAlpha(1.0);
~~~

to 

~~~cxx
// verbose is an option common to all
solver.setBooleanParameter("verbose", false);

// alpha is a osqp specific parameter, so we only set if the solver used is indeed osqp
if (solver.getSolverName() == "osqp")
{
    REQUIRE(solver.setRealNumberParameter("alpha", 1.0));
}
~~~

### Constants

Change any instance of `OsqpEigen::INFTY` to `QpSolversEigen::INFTY`, `OsqpEigen::Status` to `QpSolversEigen::Status` and `OsqpEigen::ErrorExitFlag` to `QpSolversEigen::ErrorExitFlag` .
