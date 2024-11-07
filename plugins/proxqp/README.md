# qpsolvers-eigen-proxqp

`proxqp` plugin for `qpsolvers-eigen`, the `proxqp` solver is contained in the [`proxsuite`](https://github.com/Simple-Robotics/proxsuite) library.

## Supported parameters

Unless a parameter has some qpsolvers-eigen specific notes, no documentation is reported here to avoid duplicating the official documentation. For the description of each parameter, check the official proxqp documentation:
* https://simple-robotics.github.io/proxsuite/structproxsuite_1_1proxqp_1_1Settings.html

If you need support for more parameters, please open an issue.

### Boolean parameters

| Name         | Notes                     |
|:------------:|:-------------------------:|
| `verbose`    |                           |
| `update_preconditioner`    |                |
| `compute_preconditioner` |  |
| `compute_timings` |   |
| `check_duality_gap` |  |
| `bcl_update` |  |
| `primal_infeasibility_solving` | |

### Integer parameters

| Name         | Notes                     |
|:------------:|:-------------------------:|
| `max_iter`     |                           |
| `max_iter_in `     |                           |
| `safe_guard`     |                           |
| `nb_iterative_refinement`     |                           |
| `preconditioner_max_iter`     |                           |
| `frequence_infeasibility_check` | |


### Real Number parameters

| Name         | Notes                     |
|:------------:|:-------------------------:|
| `default_mu_eq`   |     |
| `default_mu_in`   | |
| `alpha_bcl`        | |
| `beta_bcl`        | |
| `refactor_dual_feasibility_threshold` | |
| `refactor_rho_threshold` | |
| `mu_min_in` | |
| `mu_max_eq_inv` | |
| `mu_max_in_inv` | |
| `mu_update_factor` | |
| `mu_update_inv_factor` | |
| `cold_reset_mu_eq` | |
| `cold_reset_mu_in` | |
| `cold_reset_mu_eq_inv` | |
| `cold_reset_mu_in_inv` | |
| `eps_abs` | |
| `eps_rel` |  |
| `eps_refact` |  |
| `eps_duality_gap_abs` | |
| `eps_duality_gap_rel` | |
| `preconditioner_accuracy` | |
| `alpha_gpdal` | |
| `default_H_eigenvalue_estimate` | |

### String parameters

TODO
