# /// script
# requires-python = ">=3.12"
# dependencies = [
#     "casadi==3.6.6",
#     "fire",
# ]
# ///
import casadi as ca
from casadi import (
    DM,
    SX,
    CodeGenerator,
    Function,
    diag,
    log1p,
    nlpsol,
    reshape,
    sum1,
    sum2,
    vertcat,
)
from fire import Fire

# car mass and geometry
m = 230.0  # mass
wheelbase = 1.5706  # distance between the two axles
# drivetrain parameters (simplified)
C_m0 = 4.950
C_r0 = 297.030
C_r1 = 16.665
C_r2 = 0.6784
# actuator limits
T_max = 500.0
delta_max = 0.5
# general OCP parameters
Nf = 40  # horizon size
nx = 4  # state dimension
nu = 2  # control dimension
dt = 1 / 20  # sampling time

def get_continuous_dynamics_casadi() -> ca.Function:
    # state and control variables
    X = ca.SX.sym("X")
    Y = ca.SX.sym("Y")
    phi = ca.SX.sym("phi")
    v = ca.SX.sym("v")
    T = ca.SX.sym("T")
    delta = ca.SX.sym("delta")
    x = ca.vertcat(X, Y, phi, v)
    u = ca.vertcat(T, delta)

    # auxiliary variables
    beta = 0.5 * delta  # slip angle
    v_x = v * ca.cos(beta)  # longitudinal velocity
    l_R = 0.5 * wheelbase

    # assemble bicycle dynamics
    return ca.Function(
        "continuous_dynamics",
        [x, u],
        [
            ca.vertcat(
                v * ca.cos(phi + beta),
                v * ca.sin(phi + beta),
                v * ca.sin(beta) / l_R,
                (C_m0 * T - (C_r0 + C_r1 * v_x + C_r2 * v_x**2) * ca.tanh(10 * v_x))
                / m,
            )
        ],
    )


def get_discrete_dynamics_casadi() -> ca.Function:
    x = ca.SX.sym("x", nx)
    u = ca.SX.sym("u", nu)
    f = get_continuous_dynamics_casadi()
    k1 = f(x, u)
    k2 = f(x + dt / 2 * k1, u)
    k3 = f(x + dt / 2 * k2, u)
    k4 = f(x + dt * k3, u)
    return ca.Function(
        "discrete_dynamics", [x, u], [x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)]
    )

def main(filename: str = "gen", mem: bool = True, verbose: bool = True):
    # create a simple function
    x = SX.sym("x")
    y = SX.sym("y", 2, 2)
    f = Function("f", [x, y], [log1p(x) + sum1(diag(y))])
    print(f"res = {float(f(1.0, DM([[1.0, 3.0],[2.0, 4.0]]))):.10f}")

    # create a function with nlpsol
    # solver = nlpsol(
    #     "solver",
    #     "ipopt",
    #     {"f": x**2 + sum1(sum2(y**2)), "x": vertcat(x, reshape(y, 4, 1))},
    # )

    opts = {
        "main": False,
        "mex": False,
        "with_mem": mem,
        "verbose": verbose,
        "with_header": True,
        "indent": 4,
    }
    C = CodeGenerator(filename, opts)
    C.add(f)
    # C.add(get_continuous_dynamics_casadi())
    C.add(get_discrete_dynamics_casadi())
    # C.add(solver)
    C.generate()


if __name__ == "__main__":
    Fire(main)
