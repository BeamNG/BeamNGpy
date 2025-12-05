"""Newton method for root finding."""

import numpy as np

def solve(f, x0, dx, x_min, x_max, f_tol, max_iter, file=None):
    """Solve a root finding problem f(x) = 0 using a Newton method, where f: R^n -> R^n."""
    solved = False
    x = x0
    extra = None
    fx = None

    for k in range(max_iter):
        fx, extra = f(x)
        # Check element-wise tolerance
        if np.all(np.abs(fx) <= f_tol):
            reason = f"Converged after {k} iterations."
            solved = True
            break

        # Compute approximate Jacobian (using the same finite precision step)
        J = numeric_jacobian(f, x, dx, fx, x_max)

        # Solve linearized system J * delta = -f(x)
        try:
            delta = np.linalg.solve(J, -fx)
        except np.linalg.LinAlgError:
            # Fall back to gradient-like step if Jacobian is singular
            print("warning: Jacobian is singular", file=file, flush=True)
            delta = -fx

        # Make sure the step size does not exceed the variable bounds
        x_new = x + delta
        x_new = np.clip(x_new, x_min, x_max)

        x = x_new

    else:
        reason = "Maximum iterations reached without convergence."

    return x, fx, solved, reason, extra


def numeric_jacobian(f, x, dx, fx, x_max):
    """Compute a finite-difference Jacobian respecting dx."""
    n = len(x)
    J = np.zeros((n, n))
    for i in range(n):
        x_step = x.copy()
        x_step[i] += dx[i]  # forward finite difference of size dx[i]
        if x_step[i] > x_max[i]:  # make a step in the opposite direction if the step exceeds the maximum value
            x_step[i] = x[i] - dx[i]
        fx_step, _ = f(x_step)
        J[:, i] = (fx_step - fx) / (x_step[i] - x[i])
    return J
