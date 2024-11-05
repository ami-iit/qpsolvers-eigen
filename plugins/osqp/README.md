# qpsolvers-eigen-osqp

[`osqp`](https://github.com/osqp/osqp) plugin for `qpsolvers-eigen`, based on [`osqp-eigen`](https://github.com/robotology/osqp-eigen).

## Supported parameters

Unless a parameter has some qpsolvers-eigen specific notes, no documentation is reported here to avoid duplicating the official documentation. For the description of each parameter, check the official osqp documentation:
* osqp 0.6: https://osqp.org/docs/release-0.6.3/interfaces/solver_settings.html#solver-settings
* osqp 1.0: https://osqp.org/docs/interfaces/solver_settings.html

If you need support for more parameters, please open an issue.

### Boolean parameters

| Name         | Notes                     |
|:------------:|:-------------------------:|
| `polish`     |                           |
| `verbose`    |                           |
| `scaled_termination`    |                |
| `warm_start` |  `warm_start` is the name of the parameter in osqp 0.6.3, `warm_starting` in osqp 1.0.0, both are supported in qpsolvers-eigen |
| `warm_starting` |  `warm_start` is the name of the parameter in osqp 0.6.3, `warm_starting` in osqp 1.0.0, both are supported in qpsolvers-eigen |
| `adaptive_rho` |  |

### Integer parameters

| Name         | Notes                     |
|:------------:|:-------------------------:|
| `scaling`     |                           |
| `adaptive_rho_interval` |  |
| `max_iter` |  |
| `polish_refine_iter` | | 
| `linsys_solver` | Documentation available at https://osqp.org/docs/release-0.6.3/interfaces/solver_settings.html#solver-settings (osqp 0.6) https://osqp.org/docs/interfaces/linear_systems_solvers.html#linear-system-solvers-setting (osqp 1.0) |
| `check_termination` | |

### Real Number parameters

| Name         | Notes                     |
|:------------:|:-------------------------:|
| `rho`     |                           |
| `sigma`   | |
| `adaptive_rho_tolerance`        | |
| `adaptive_rho_fraction`        | |
| `eps_abs` | |
| `eps_rel` | |
| `eps_prim_inf` | |
| `eps_dual_inf` | |
| `alpha` |  |
| `delta` |  |
