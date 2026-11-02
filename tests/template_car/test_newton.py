"""Newton root finder used by the template car optimization."""

from __future__ import annotations

import io

import numpy as np
import pytest

from beamngpy.tools.template_car.newton import numeric_jacobian, solve


def _solve(fun, x0, *, dx=1e-3, x_min=-10.0, x_max=10.0, f_tol=1e-9, max_iter=50, file=None, evaluated=None):
    """Run solve() on ``fun`` with scalar settings broadcast to every variable.

    ``extra`` of every evaluation is the evaluated point, which is also appended to ``evaluated``.
    """
    x0 = np.atleast_1d(np.asarray(x0, dtype=np.float64))

    def full(value):
        return np.full(len(x0), value, dtype=np.float64)

    def f(x):
        if evaluated is not None:
            evaluated.append(x.copy())
        return np.atleast_1d(np.asarray(fun(x), dtype=np.float64)), x.copy()

    return solve(f, x0, full(dx), full(x_min), full(x_max), full(f_tol), max_iter, file=file)


def test_linear_converges_in_one_step():
    x, fx, solved, reason, _ = _solve(lambda x: 3 * x - 6, [0.0])
    assert solved
    assert reason == "Converged after 1 iterations."
    assert x == pytest.approx([2.0])
    assert abs(fx[0]) <= 1e-9


def test_initial_root_needs_no_iteration():
    x, _, solved, reason, extra = _solve(lambda x: x - 1, [1.0])
    assert solved
    assert reason == "Converged after 0 iterations."
    np.testing.assert_array_equal(x, [1.0])
    np.testing.assert_array_equal(extra, [1.0])


def test_nonlinear_scalar():
    x, _, solved, _, _ = _solve(lambda x: x**2 - 2, [1.0], dx=1e-6)
    assert solved
    assert x == pytest.approx([np.sqrt(2)], abs=1e-8)


def test_coupled_system():
    def fun(v):
        x, y = v
        return [x**2 + y**2 - 2, x - y]

    x, _, solved, _, _ = _solve(fun, [2.0, 0.5], dx=1e-6)
    assert solved
    assert x == pytest.approx([1.0, 1.0], abs=1e-6)


def test_root_outside_bounds_is_not_solved():
    evaluated = []
    x, _, solved, reason, _ = _solve(lambda x: x**2 - 100.0001, [5.0], max_iter=5, evaluated=evaluated)
    assert not solved
    assert reason == "Maximum iterations reached without convergence."
    np.testing.assert_array_equal(x, [10.0])
    assert max(point[0] for point in evaluated) <= 10.0


def test_singular_jacobian_falls_back_to_gradient_step():
    log = io.StringIO()
    # The second variable has no influence, so the Jacobian has a zero column.
    x, _, solved, _, _ = _solve(lambda v: [v[0] - 1, v[0] - 1], [3.0, 0.5], file=log)
    assert "warning: Jacobian is singular" in log.getvalue()
    assert solved
    # The fallback step is -f(x): (3, 0.5) - (2, 2)
    assert x == pytest.approx([1.0, -1.5])


def _linear_probe(calls):
    def f(x):
        calls.append(x.copy())
        return np.array([2 * x[0], 3 * x[0] + 5 * x[1]]), None

    return f


def test_jacobian_uses_forward_step():
    calls = []
    f = _linear_probe(calls)
    x = np.array([1.0, 2.0])
    J = numeric_jacobian(f, x, np.array([0.1, 0.1]), f(x)[0], np.array([5.0, 5.0]))
    np.testing.assert_allclose(J, [[2.0, 0.0], [3.0, 5.0]])
    np.testing.assert_allclose(calls[1:], [[1.1, 2.0], [1.0, 2.1]])


def test_jacobian_steps_backward_at_upper_bound():
    calls = []
    f = _linear_probe(calls)
    x = np.array([1.0, 2.0])
    J = numeric_jacobian(f, x, np.array([0.1, 0.1]), f(x)[0], np.array([1.05, 5.0]))
    np.testing.assert_allclose(J, [[2.0, 0.0], [3.0, 5.0]])
    np.testing.assert_allclose(calls[1:], [[0.9, 2.0], [1.0, 2.1]])


def test_extra_comes_from_solution_evaluation():
    x, _, solved, _, extra = _solve(lambda v: [v[0] ** 3 - 8, v[1] - 0.25], [1.0, 1.0], dx=1e-6)
    assert solved
    np.testing.assert_array_equal(extra, x)
