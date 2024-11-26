import numpy as np


class RungeKuttaStep:
    def __init__(self,m_ode,  adaptive_step=False):
        # Weights for the 4th-order accurate solution
        self.m_b = np.array([47./450., 0., 12./25., 32./225., 1./30., 6./25.])
        # Weights for the 3rd-order embedded solution
        self.m_b_e = np.array([1./9., 0., 9./20., 16./45., 1./12., 0.])
        self.m_c = np.array([0., 2. / 9., 1. / 3., 3. / 4., 1., 5. / 6.]) # Coefficients for intermediate stages
        self.m_A = np.array([[0.,      0.,        0.,      0.,     0.,     0.],
                             [2./9.,    0.,        0.,      0.,     0.,     0.],
                             [1./12.,   1./4.,      0.,      0.,     0.,     0.],
                             [69./128., -243./128., 135./64., 0.,     0.,     0.],
                             [-17./12., 27./4.,     -27./5.,  16./15., 0.,     0.],
                             [65./432., -5./16.,    13./16.,  4./27.,  5./144., 0. ]]) #Matrix \f$ \mathbf{A} \f$ (lower triangular matrix).
        self.m_adaptive_step = adaptive_step
        self.m_A_tol =  1e-6  # Absolute tolerance
        self.m_R_tol = 1e-4  # Relative tolerance
        self.m_order = 4  # Order of the method
        self.m_fac =0.9 #Safety factor for adaptive step.
        self.m_fac_max = 1.5  # Maximum safety factor for adaptive step.
        self.m_fac_min = 0.2  #Minimum safety factor for adaptive step.
        self.m_ode = m_ode  # ODE system object with methods f (explicit evaluation)

    def stepRKF45(self, x_k, x_dot_k, t_k, d_t):
        """
        Perform a single integration step.

        Parameters:
            x_k: Current state vector.
            x_dot_k: Current derivative vector.
            t_k: Current time.
            d_t: Current time step.

        Returns:
            x_out: State vector at the next step.
            x_dot_out: Derivative vector at the next step.
            d_t_star: Suggested time step for the next step.
            ierr: Error code (0 if successful, >0 if an error occurs).
        """
        # Solve the system to obtain K
        K = self.solve_step(x_k, t_k, d_t)

        # Suggested time step for the next advancing step
        d_t_star = d_t

        # Error code check
        # if ierr > 0:
        #     x_out = np.full_like(x_k, np.nan)
        #     x_dot_out = np.full_like(x_dot_k, np.nan)
        #     return x_out, x_dot_out, d_t_star, ierr

        # Perform the step and obtain x_k+1
        x_out = x_k + d_t * K @ self.m_b

        # Extract x_dot_k+1 from K (last column of K)
        x_dot_out = K[:, -1]

        # Adapt next time step
        if self.m_adaptive_step and self.m_b_e is not None:
            x_e = x_k + d_t * K @ self.m_b_e
            d_t_star = self.adapt_step(x_out, x_e, d_t_star)

        return x_out, x_dot_out, d_t_star

    def step_node(self, i, x_k, K, d_t):
        """
        Compute the intermediate node for the Runge-Kutta method.

        Parameters:
            i: Current stage index (1-based index)
            x_k: State at the current step
            K: Matrix of stage derivatives
            d_t: Time step

        Returns:
            out: Intermediate node value
        """
        # Initialize the node computation
        out = np.zeros_like(x_k)

        # Accumulate contributions from previous stages
        for j in range(i - 1):  # Python uses 0-based indexing
            out += self.m_A[i - 1, j] * K[:, j]  # Adjust indices for 0-based indexing

        # Compute the node
        out = x_k + out * d_t
        return out

    def solve_step(self, x_k, t_k, d_t):
        """
        Solve the system to compute the stage values K.

        Parameters:
            x_k: Current state vector.
            t_k: Current time.
            d_t: Time step.

        Returns:
            out: Computed K matrix (each column corresponds to a stage).
            ierr: Error code (0 if successful, >0 if an error occurs).
        """
        n_states = len(x_k)
        nc = len(self.m_c)  # Number of stages nodi
        K = np.zeros((n_states, nc))  # Initialize K matrix by replicating K_0

        for i in range(nc):
            # Compute the node state
            x_i = self.step_node(i, x_k, K, d_t)
            # Explicit case: directly compute K[:, i]
            print(self.m_ode.f(x_i, t_k + self.m_c[i] * d_t))
            K[:, i] = self.m_ode.f(x_i, t_k + self.m_c[i] * d_t)
        return K

    def adapt_step(self, x_h, x_l, d_t):
        """
        Adapt the time step based on the error between high and low-order approximations.

        Parameters:
            x_h: High-order approximation
            x_l: Low-order approximation
            d_t: Current time step

        Returns:
            out: Suggested time step for the next iteration
        """
        # Compute the error with 2-norm
        r = (x_h - x_l) / (self.m_A_tol + 0.5 * self.m_R_tol * (np.abs(x_h) + np.abs(x_l)))
        e = np.linalg.norm(r, 2) / len(x_h)

        # Compute the suggested time step
        q = self.m_order + 1
        out = d_t * min(self.m_fac_max, max(self.m_fac_min, self.m_fac * (1 / e) ** (1 / q)))

        return out

class MyODE:

    def f(self, x, t):
        return -np.sin(x)  # Example ODE function



if __name__ == '__main__':

    # Example instantiation
    rk = RungeKuttaStep( m_ode=MyODE(),   adaptive_step=False)

    x_k = np.array([1.0, 2.0])
    x_dot_k = np.array([0.0, 0.0])
    t_k = 0.0
    d_t = 0.1

    x_out, x_dot_out, d_t_star = rk.stepRKF45(x_k, x_dot_k, t_k, d_t)
    print(f"x_out: {x_out}, x_dot_out: {x_dot_out}, d_t_star: {d_t_star}")