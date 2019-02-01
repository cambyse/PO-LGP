import numpy as np

class MotionProblem:
    def __init__(self):
        self.task_maps = []
        self.dim = 0

    def add_task(self, task, wpath, start, end):
        self.task_maps.append((wpath, start, end, task))
        self.dim += task.dim

    def gamma(self, x):
        n_steps = x.shape[0]
        x_dim = x.shape[1]

        gamma = np.zeros(self.dim * n_steps)
        Jgamma = np.zeros((self.dim * n_steps, n_steps * x_dim))

        col_offset = 0
        for path, start, end, task in self.task_maps:
            if path is None:
                path = [(i, 1.0) for i in range(n_steps)]
            self.fill_gamma(x, task, col_offset, path, start, end, gamma, Jgamma)
            col_offset += task.dim

        return gamma, Jgamma

    def get_step(self, wpath, s):
        if s < 0:
            assert False
        if s >= len(wpath):
            return wpath[-1]
        return wpath[s]

    def fill_gamma(self, x, task, dim_offset, wpath, start, end, gamma, Jgamma):
        n_steps = x.shape[0]
        x_dim = x.shape[1]

        if end == -1:
            end = n_steps

        if task.order == 0:
            for _s in range(start, end):
                t, w = self.get_step(wpath, _s)
                phi, Jphi = task.phi(x[t])
                for dim_index in range(task.dim):
                    i = self.dim*t+dim_offset+dim_index
                    gamma[i] = w * phi[dim_index]
                    for k in range(x_dim):
                        Jgamma[i,x_dim*t+k] = w * Jphi[dim_index, k]
        elif task.order == 1:
            for _s in range(start+1, end):
                t, w = self.get_step(wpath, _s)
                tm1, _ = self.get_step(wpath, _s-1)
                phi, Jphi = task.phi(x[t]-x[tm1])
                for dim_index in range(task.dim):
                    i = self.dim * t + dim_offset + dim_index
                    gamma[i] = w * phi[dim_index]
                    for k in range(x_dim):
                        Jgamma[i,x_dim*(tm1)+k] = -w * Jphi[dim_index, k]
                        Jgamma[i,x_dim*t+k] = w * Jphi[dim_index, k]
        elif task.order == 2:
            for _s in range(start+1, end-1):
                t, w = self.get_step(wpath, _s)
                tm1, _ = self.get_step(wpath, _s - 1)
                tp1, _ = self.get_step(wpath, _s + 1)
                phi, Jphi = task.phi(x[tm1] - 2 * x[t] + x[tp1])
                for dim_index in range(task.dim):
                    i = self.dim * t + dim_offset + dim_index
                    gamma[i] = w * phi[dim_index]
                    for k in range(x_dim):
                        Jgamma[i,x_dim*(tm1)+k] = w * Jphi[dim_index, k]
                        Jgamma[i,x_dim*t+k] = - w * 2 * Jphi[dim_index, k]
                        Jgamma[i,x_dim*(tp1)+k] = w * Jphi[dim_index, k]

    def traj_cost(self, x):
        c, _ = self.gamma(x)
        return np.dot(c, c)

class PyKOMO:
    def __init__(self):
        self.motion_problem = MotionProblem()
        self.lambda_0 = 0.1
        self.rho = 0.01
        self.eps = 0.01
        self.n_steps = 1

    def add_task(self, task, wpath=None, start=0, end=-1):
        if wpath is None:
            if end == -1:
                end=self.n_steps
            wpath = [(i, 1.0) for i in range(start, self.n_steps)]
        self.motion_problem.add_task(task, wpath, start, end)

    def traj_cost(self, x):
        return self.motion_problem.traj_cost(x)

    def set_n_steps(self, n):
        self.n_steps = n

    def run(self, x0):
        _lambda = self.lambda_0
        _alpha = 1

        x = self.get_init(x0)
        x_flat = np.ndarray.flatten(x)
        I = np.identity(x_flat.shape[0])
        while True:
            gamma, Jgamma = self.motion_problem.gamma(x)
            JgammaT = np.transpose(Jgamma)
            A = 2 * np.matmul(JgammaT, Jgamma) + _lambda * I
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

    def get_init(self, x0):
        x = np.zeros((self.n_steps, x0.shape[0]))
        for i in range(self.n_steps):
            x[i,:]=x0
        return x