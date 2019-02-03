import numpy as np
from task_map import TaskMapType

def is_semi_pos_def(m):
    eigvals = np.linalg.eigvals(m)
    return np.all( eigvals >= 0)

class MotionProblem:
    def __init__(self):
        self.task_maps = []
        self.dim = 0

    def add_task(self, task, wpath, start, end):
        self.task_maps.append((wpath, start, end, task))
        self.dim += task.dim

    def gamma(self, x, mu=1, filter=[TaskMapType.COST, TaskMapType.EQ]): #mu = additional coefficient for constraint (square penalty)
        n_steps = x.shape[0]
        x_dim = x.shape[1]

        gamma = np.zeros(self.dim * n_steps)
        Jgamma = np.zeros((self.dim * n_steps, n_steps * x_dim))

        col_offset = 0
        for path, start, end, task in self.task_maps:
            if task.type in filter:
                if path is None:
                    path = [(i, 1.0) for i in range(n_steps)]
                if end == -1:
                    end = len(path)
                self.fill_gamma(x, task, col_offset, path, start, end, mu, gamma, Jgamma)
            col_offset += task.dim

        return gamma, Jgamma

    def get_step(self, wpath, s):
        if s < 0:
            return None, None
        if s >= len(wpath):
            return None, None
        return wpath[s]

    def fill_gamma(self, x, task, dim_offset, wpath, start, end, mu, gamma, Jgamma):
        n_steps = x.shape[0]
        x_dim = x.shape[1]

        for phase in range(start, end):
            # get time steps
            tm1, _ = self.get_step(wpath, phase - 1)
            t, w = self.get_step(wpath, phase)
            tp1, _ = self.get_step(wpath, phase + 1)
            context = self.extract_context(tm1, t, tp1, x)
            if task.type == TaskMapType.EQ:
                w = w*mu
            if task.order == 0:
                phi, Jphi = task.phi(x[t], context)
                for dim_index in range(task.dim):
                    i = n_steps * (dim_offset + dim_index) + t
                    gamma[i] = w * phi[dim_index]
                    for k in range(x_dim):
                        Jgamma[i, x_dim * t + k] = w * Jphi[dim_index, k]
            elif task.order == 1:
                if tm1 is not None:
                    phi, Jphi = task.phi(x[t]-x[tm1], context)
                    for dim_index in range(task.dim):
                        i = n_steps * (dim_offset + dim_index) + t
                        gamma[i] = w * phi[dim_index]
                        for k in range(x_dim):
                            Jgamma[i, x_dim * tm1 + k] =-w * Jphi[dim_index, k]
                            Jgamma[i, x_dim * t   + k] = w * Jphi[dim_index, k]
            elif task.order == 2:
                if tm1 is not None and tp1 is not None:
                    phi, Jphi = task.phi(x[tm1] - 2 * x[t] + x[tp1], context)
                    for dim_index in range(task.dim):
                        i = n_steps * (dim_offset + dim_index) + t
                        gamma[i] = w * phi[dim_index]
                        for k in range(x_dim):
                            Jgamma[i, x_dim * tm1+k] = w * Jphi[dim_index, k]
                            Jgamma[i, x_dim * t  +k] =-w * 2 * Jphi[dim_index, k]
                            Jgamma[i, x_dim * tp1+k] = w * Jphi[dim_index, k]

    def extract_context(self, tm1, t, tp1, x):
        context = [None, None, None]
        for i, _t in enumerate([tm1, t, tp1]):
            if _t is not None:
                context[i] = x[_t]
        return context

    def traj_cost(self, x):
        c, _ = self.gamma(x, filter=[TaskMapType.COST])
        return np.dot(c, c)

    def equality_constraint(self, x):
        c, _ = self.gamma(x, filter=[TaskMapType.EQ])
        return np.dot(c, c)

class PyKOMO:
    def __init__(self):
        self.motion_problem = MotionProblem()
        self.lambda_0 = 0.1
        self.rho = 0.01
        self.eps = 0.01 #update size
        self.eps_h = 0.001 #max constraint violation
        self.n_phases = 1

    def add_task(self, task, wpath=None, start=0, end=-1): #end not included
        if wpath is None:
            wpath = [(i, 1.0) for i in range(0, self.n_phases)]
        if end == -1:
            end = len(wpath)
        self.motion_problem.add_task(task, wpath, start, end)

    def traj_cost(self, x):
        return self.motion_problem.traj_cost(x)

    def equality_constraint(self, x):
        return self.motion_problem.equality_constraint(x)

    def set_n_phases(self, n):
        self.n_phases = n

    def run(self, x0, initial=None):
        return self.run_gauss_newton_with_square_penalty(x0, initial)

    def run_gauss_newton_with_square_penalty(self, x0, initial=None):
        current_mu = 1.0
        x = self.run_gauss_newton(x0, initial, mu=current_mu)
        constraint_violation = self.equality_constraint(x)

        while constraint_violation > self.eps_h:
            current_mu *= 10
            x = self.run_gauss_newton(x0, initial=x, mu=current_mu)
            constraint_violation = self.equality_constraint(x)
        return x

    def run_gauss_newton(self, x0, initial=None, mu=1):
        _lambda = self.lambda_0
        _alpha = 1

        x = self.get_init(x0, override=initial)
        x_flat = np.ndarray.flatten(x)
        I = np.identity(x_flat.shape[0])
        while True:
            gamma, Jgamma = self.motion_problem.gamma(x, mu=mu)
            JgammaT = np.transpose(Jgamma)
            approxHessian = 2 * np.matmul(JgammaT, Jgamma)
            #print(is_semi_pos_def(approxHessian))
            A = approxHessian + _lambda * I
            B = 2 * np.matmul(JgammaT, gamma)
            d = np.linalg.solve(A, -B)

            while self.traj_cost(np.reshape(x_flat + _alpha * d, x.shape)) > self.traj_cost(x) + self.rho * np.matmul(np.transpose(B),
                                                                             _alpha * d):  # line search
                _alpha = _alpha * 0.5

            x_flat = x_flat + _alpha * d
            x = np.reshape(x_flat, x.shape)
            _alpha = 1
            if np.linalg.norm(_alpha * d) < self.eps:
                break

        return x

    def get_init(self, x0, override=None):
        if override is not None:
            return override

        x = np.zeros((self.n_phases, x0.shape[0]))
        for i in range(self.n_phases):
            x[i,:]=x0
        return x