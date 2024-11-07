# qpsolvers-eigen

# osqp-eigen

Simple C++ abstraction layer for quadratic programming solvers using [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page).

## 🛠️ Usage

Please install the library following one (and just one) method listed below.

#### 📦 Install with conda or pixi (recommended)

> [!WARNING]
> This section is just a preview

You can easily the library with [`conda`](https://github.com/conda-forge/qpsolvers-eigen-feedstock) in a new conda environment with
```
conda create -n newenvname -c conda-forge qpsolvers-eigen
```
`conda` will automatically install all the supported dependencies.

To add qpsolvers-eigen to a `pixi` project, just run:

```
pixi add qpsolvers-eigen
```

#### ⚙️ Install via build from source for internal development (advanced)

If you just want to modify `qpsolvers-eigen` and run the tests again your modification,
the easiest way to do that is to use [`pixi`](https://pixi.sh/latest/), in particular you just need to run:

~~~
git clone https://github.com/ami-iit/qpsolvers-eigen.git
cd qpsolvers-eigen
pixi run test
~~~

#### ⚙️ Build from source (advanced)

If you want to use a package manager that does not provide `qpsolvers-eigen` packages, youc can do that
as qpsolvers-eigen is a fairly standard CMake project. To do that, first of all install either via a package
manager or manually the following depencies:

Required dependencies:
* [eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
* [cmake](https://cmake.org/)
* C and C++ compiler

Optional dependencies:
* [sharedlibpp](https://github.com/ami-iit/sharedlibpp) (if not found an internal copy is used and installed)
* [osqp-eigen](https://github.com/robotology/osqp-eigen) (if not found the `osqp` plugin is not compiled)
* [proxsuite](https://github.com/Simple-Robotics/proxsuite) (if not found the `proxqp` plugin is not compiled)

Test dependencies:
* [catch2](https://github.com/catchorg/Catch2)

Then follow the instructions

1. Clone the repository
   ```
   git clone https://github.com/ami-iit/qpsolvers-eigen.git
   ```
2. Build it
   ```
   cd osqp-eigen
   mkdir build
   cd build
   cmake -GNinja -DCMAKE_INSTALL_PREFIX:PATH=<custom-folder> ../
   ninja
   ninja install
   ```
3. Add the following environmental variable to ensure that `find_package(QpSolversEigen REQUIRED)` is successful:
   ```
   QpSolversEigen_DIR=/path/where/you/installed/
   ```

## 🖥️ How to use the library

**qpsolvers-eigen** provides native `CMake` support which allows the library to be easily used in `CMake` projects.

**qpsolvers-eigen** exports a CMake target called `QpSolversEigen::QpSolversEigen` which can be imported using the `find_package` CMake command and used by calling `target_link_libraries` as in the following example:

```cmake
cmake_minimum_required(VERSION 3.16)
project(myproject)
find_package(QpSolversEigen REQUIRED)
add_executable(example example.cpp)
target_link_libraries(example QpSolversEigen::QpSolversEigen)
```

A minimal `example.cpp` is:

~~~cxx
#include <cstdlib>
#include <iostream>

#include <QpSolversEigen/QpSolversEigen.hpp>
#include <Eigen/Dense>
#include <Eigen/Sparse>


int main()
{
    Eigen::SparseMatrix<double> H_s(2, 2);
    H_s.insert(0, 0) = 4;
    H_s.insert(0, 1) = 1;
    H_s.insert(1, 0) = 1;
    H_s.insert(1, 1) = 2;

    Eigen::SparseMatrix<double> A_s(3, 2);
    A_s.insert(0, 0) = 1;
    A_s.insert(0, 1) = 1;
    A_s.insert(1, 0) = 1;
    A_s.insert(2, 1) = 1;

    Eigen::Matrix<double, 2, 1> gradient;
    gradient << 1, 1;

    Eigen::Matrix<double, 3, 1> lowerBound;
    lowerBound << 1, 0, 0;

    Eigen::Matrix<double, 3, 1> upperBound;
    upperBound << 1, 0.7, 0.7;

    QpSolversEigen::Solver solver;

    // Here you select the solver, possible options are:
    // * osqp
    // * proxqp
    bool ok = solver.instantiateSolver("osqp");

    if (!ok)
    {
        std::cerr << "Error in instantiating the solver" << std::endl;
        return EXIT_FAILURE;
    }

    // Set osqp-specific parameters
    if (solver.getSolverName() == "osqp")
    {
        solver.setBooleanParameter("verbose", true);
        solver.setRealNumberParameter("alpha", 1.0);
        // See https://github.com/robotology/osqp-eigen/pull/172
        solver.setBooleanParameter("polish", true);
    }

    solver.data()->setNumberOfVariables(2);
    solver.data()->setNumberOfConstraints(3);
    ok = ok && solver.data()->setHessianMatrix(H_s);
    ok = ok && solver.data()->setGradient(gradient);
    ok = ok && solver.data()->setLinearConstraintsMatrix(A_s);
    ok = ok && solver.data()->setLowerBound(lowerBound);
    ok = ok && solver.data()->setUpperBound(upperBound);
    ok = ok && solver.initSolver();

    if (!ok)
    {
        std::cerr << "Error in solver initialization" << std::endl;
        return EXIT_FAILURE;
    }

    ok = ok && (solver.solveProblem() == QpSolversEigen::ErrorExitFlag::NoError);

    if (!ok)
    {
        std::cerr << "Error in solving the problem" << std::endl;
        return EXIT_FAILURE;
    }

    std::cerr << "Solution: " << solver.getSolution() << std::endl;
}
~~~

For more examples, check the content of the [./examples](./examples) folder in this repo.


### Migrate from osqp-eigen

If you are already using `osqp-eigen` and you want to understand how to migrate your code to `qpsolvers-eigen`, check the [`./doc/migrate_from_osqp_eigen.md`](./doc/migrate_from_osqp_eigen.md) document.

## 🐛 Bug reports and support

All types of [issues](https://github.com/ami-iit/qpsolvers-eigen/issues/new) are welcome.

## 📝 License

Materials in this repository are distributed under the following license:

> All software is licensed under the BSD 3-Clause License. See [LICENSE](https://github.com/robotology/osqp-eigen/blob/master/LICENSE) file for details.
