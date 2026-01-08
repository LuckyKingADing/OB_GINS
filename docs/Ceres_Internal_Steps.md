# Ceres Solver: Internal Execution Steps

This document details the internal lifecycle of a call to `ceres::Solver::Solve`. It is useful for understanding performance bottlenecks and the sequence of operations in the Generalized GNSS/INS (OB_GINS) optimization.

## 1. Preprocessing Phase
The preprocessing phase prepares the problem for the efficient iterative minimization.

*   **Problem Canonicalization**:
    *   The user's `ceres::Problem` is converted into a `ceres::internal::Program`.
    *   **Removals**: Parameter blocks marked constant (`problem.SetParameterBlockConstant`) are removed from the vector of active variables.
    *   **Indirection**: An "effective" parameter map is created, mapping user memory pointers to indices in the compact solver state vector $x$.

*   **Ordering & Analysis**:
    *   **Parameter Ordering**: The solver computes an elimination ordering for the parameter blocks.
        *   *Context*: For `SPARSE_NORMAL_CHOLESKY` (used in OB_GINS), an Approximate Minimum Degree (AMD) ordering is typically computed to minimize fill-in during Cholesky factorization.
    *   **Sparsity Analysis**: The Jacobian's sparsity pattern is analyzed to pre-allocate Compressed Row Storage (CRS) structures. This ensures that the Jacobian memory layout is static during the solving phase.

*   **Evaluator Setup**:
    *   An `Evaluator` chain is built (handling `CostFunctor` calls, loss functions, and local parameterizations/manifolds).

## 2. Minimizer Loop (Levenberg-Marquardt)
The core optimization happens in `TrustRegionMinimizer`.

### Iteration Cycle
1.  **Evaluation**:
    *   Compute Residuals $F(x)$ and Jacobian $J(x)$.
    *   Apply `LossFunction` corrections (scaling residuals and Jacobian rows).

2.  **Linear System Formulation**:
    *   Construct the augmented Normal Equations for the step $\Delta x$:
        $$ (H + \mu I) \Delta x = -g $$
        *   $H = J^T J$ (approximate Hessian)
        *   $g = J^T F$ (Gradient)
        *   $\mu$: Levenberg-Marquardt damping parameter (inverse of Trust Region radius).

3.  **Linear Solve**:
    *   Execute the configured linear solver (`SPARSE_NORMAL_CHOLESKY`).
    *   Factorize the matrix $(H + \mu I)$ using Simplicial Cholesky decomposition.
    *   Solve for $\Delta x$.

4.  **Step Assessment**:
    *   **Trial**: Compute $x_{trial} = x \boxplus \Delta x$ (using `Manifold` if defined, otherwise vector addition).
    *   **Check**: Evaluate Cost($x_{trial}$).
    *   **Ratio ($\rho$)**: Compute relative decrease $\rho = \frac{\text{Actual Decrease}}{\text{Model Decrease}}$.

5.  **Update Rule**:
    *   **If $\rho > \epsilon$ (Successful)**:
        *   Accept update: $x \leftarrow x_{trial}$.
        *   Decrease $\mu$ (`max(mu * 0.33, 1e-16)`).
    *   **If $\rho \le \epsilon$ (Unsuccessful)**:
        *   Reject update (keep old $x$).
        *   Increase $\mu$ (`mu * 2` or similar scheme).

## 3. Post-processing Phase
After the loop terminates (due to convergence tolerances or max iterations):

*   **State Writeback**:
    *   The final optimized values from the internal state vector $x$ are copied back to the double pointers provided by the user in `problem.AddParameterBlock`.
    *   This is the **only** time user memory is modified (unless `update_state_every_iteration` is true).

*   **Reporting**:
    *   A `Solver::Summary` object is populated with timing breakdown (`preprocessor_time`, `minimizer_time`) and final costs.
