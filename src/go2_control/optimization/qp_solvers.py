import numpy as np
import osqp
import proxsuite
from scipy import sparse

def as_dense(matrix):
    if sparse.issparse(matrix):
        return matrix.toarray()

    return np.asarray(matrix)

def solve_with_osqp(
    quadratic_cost,
    linear_cost,
    equality_matrix,
    equality_target,
    inequality_matrix,
    inequality_lower_bound,
    inequality_upper_bound,
):
    constraint_matrix = sparse.vstack(
        [
            sparse.csc_matrix(equality_matrix),
            sparse.csc_matrix(inequality_matrix),
        ],
        format="csc",
    )
    lower_bound = np.concatenate(
        [
            equality_target,
            inequality_lower_bound,
        ]
    )
    upper_bound = np.concatenate(
        [
            equality_target,
            inequality_upper_bound,
        ]
    )

    solver = osqp.OSQP()
    solver.setup(
        P=sparse.csc_matrix(quadratic_cost),
        q=linear_cost,
        A=constraint_matrix,
        l=lower_bound,
        u=upper_bound,
        verbose=False,
        polishing=True,
    )

    result = solver.solve()

    if not result.info.status.lower().startswith("solved"):
        raise RuntimeError(
            f"OSQP failed: {result.info.status}"
        )

    return result.x.copy(), result.info



def solve_with_proxqp(
    quadratic_cost,
    linear_cost,
    equality_matrix,
    equality_target,
    inequality_matrix,
    inequality_lower_bound,
    inequality_upper_bound,
):
    number_of_variables = linear_cost.size
    number_of_equalities = equality_matrix.shape[0]
    number_of_inequalities = inequality_matrix.shape[0]

    solver = proxsuite.proxqp.dense.QP(
        number_of_variables,
        number_of_equalities,
        number_of_inequalities,
    )

    solver.init(
        as_dense(quadratic_cost),
        linear_cost,
        as_dense(equality_matrix),
        equality_target,
        as_dense(inequality_matrix),
        inequality_lower_bound,
        inequality_upper_bound,
    )

    solver.solve()

    status = str(solver.results.info.status)

    if "SOLVED" not in status:
        raise RuntimeError(
            f"ProxQP failed: {status}"
        )

    return (
        solver.results.x.copy(),
        solver.results.info,
    )