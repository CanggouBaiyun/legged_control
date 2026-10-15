import numpy as np
import osqp
import proxsuite
from scipy import sparse

def as_dense(matrix):
    if sparse.issparse(matrix):
        return matrix.toarray()

    return np.asarray(matrix)

class OSQPWorkspace:
    """Reuse an OSQP solver while the matrix structure stays unchanged."""

    def __init__(self):
        self.solver = None
        self.matrix_pattern = None
        self.setup_count = 0
        self.update_count = 0

    def solve(self, P, q, A, l, u):
        # OSQP 只使用二次代价矩阵的上三角部分。
        P = sparse.triu(P, format="csc")
        A = sparse.csc_matrix(A, copy=True)

        # 保证同一稀疏结构具有一致的数据排列。
        P.sum_duplicates()
        P.sort_indices()
        A.sum_duplicates()
        A.sort_indices()

        # 检查矩阵尺寸和非零元素所在位置。
        matrix_pattern = (
            P.shape,
            P.indptr.tobytes(),
            P.indices.tobytes(),
            A.shape,
            A.indptr.tobytes(),
            A.indices.tobytes(),
        )

        if (
            self.solver is None
            or matrix_pattern != self.matrix_pattern
        ):
            # 第一次调用，或者矩阵结构发生变化时，重新 setup。
            solver = osqp.OSQP(algebra="builtin")
            solver.setup(
                P=P,
                q=q,
                A=A,
                l=l,
                u=u,
                verbose=False,
                polishing=True,
                max_iter=20000,
                warm_starting=False,
            )

            self.solver = solver
            self.matrix_pattern = matrix_pattern
            self.setup_count += 1
        else:
            # 结构相同时，更新数值，复用已有求解器。
            self.solver.update(
                Px=P.data,
                Ax=A.data,
                q=q,
                l=l,
                u=u,
            )
            self.update_count += 1

        return self.solver.solve()

def solve_with_osqp(
    quadratic_cost,
    linear_cost,
    equality_matrix,
    equality_target,
    inequality_matrix,
    inequality_lower_bound,
    inequality_upper_bound,
    workspace=None,
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

    if workspace is None:
        workspace = OSQPWorkspace()

    result = workspace.solve(
        P=quadratic_cost,
        q=linear_cost,
        A=constraint_matrix,
        l=lower_bound,
        u=upper_bound,
    )

    if result.info.status.lower() != "solved":
        print(
            "[OSQP diagnostic] "
            f"status={result.info.status}, "
            f"iterations={result.info.iter}, "
            f"primal_residual={result.info.prim_res:.3e}, "
            f"dual_residual={result.info.dual_res:.3e}"
        )

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