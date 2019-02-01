import numpy as np

class MotionProblem:
    def __init__(self):
        self.task_maps = []
        self.dim = 0

    def add_task(self, task):
        self.task_maps.append(task)
        self.dim += task.dim

    def gamma(self, x):
        n_steps = x.shape[0]
        x_dim = x.shape[1]

        gamma = np.zeros(self.dim * n_steps)
        Jgamma = np.zeros((self.dim * n_steps, n_steps * x_dim))

        col_offset = 0
        for task in self.task_maps:
            self.fill_gamma(x, task, col_offset, gamma, Jgamma)
            col_offset += task.dim

        return gamma, Jgamma

    def fill_gamma(self, x, task, dim_offset, gamma, Jgamma):
        n_steps = x.shape[0]
        x_dim = x.shape[1]

        if task.order == 0:
            for s in range(0, n_steps):
                phi, Jphi = task.phi(x[s])
                for dim_index in range(task.dim):
                    i = self.dim*s+dim_offset+dim_index
                    gamma[i] = phi[dim_index]
                    for k in range(x_dim):
                        Jgamma[i,x_dim*s+k] += Jphi[dim_index, k]
        elif task.order == 1:
            for s in range(1, n_steps):
                phi, Jphi = task.phi(x[s]-x[s-1])
                for dim_index in range(task.dim):
                    i = self.dim * s + dim_offset + dim_index
                    gamma[i] = phi[dim_index]
                    for k in range(x_dim):
                        Jgamma[i,x_dim*(s-1)+k] = -Jphi[dim_index, k]
                        Jgamma[i,x_dim*s+k] = Jphi[dim_index, k]
        elif task.order == 2:
            for s in range(1, n_steps-1):
                phi, Jphi = task.phi(x[s-1] - 2 * x[s] + x[s+1])
                for dim_index in range(task.dim):
                    i = self.dim * s + dim_offset + dim_index
                    gamma[i] = phi[dim_index]
                    for k in range(x_dim):
                        Jgamma[i,x_dim*(s-1)+k] = Jphi[dim_index, k]
                        Jgamma[i,x_dim*s+k] = -2 * Jphi[dim_index, k]
                        Jgamma[i,x_dim*(s+1)+k] = Jphi[dim_index, k]

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

    def add_task(self, task):
        self.motion_problem.add_task(task)

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