import numpy as np

v_desired = 1
acc_coef = 0

class TaskMap:
    def __init__(self, name = '', order = 0, dim = 1):
        self.order = order
        self.name = name
        self.dim = dim

    def gamma(self, x):
        n = x.shape[0]
        gamma = np.zeros(n)
        Jgamma = np.zeros((n, n))
        if self.order == 0:
            for i in range(0, n):
                phi, Jphi = self.phi(x[i])
                gamma[i] = phi
                Jgamma[i,i] = Jphi
        elif self.order == 1:
            for i in range(1, n):
                phi, Jphi = self.phi(x[i-1],x[i])
                gamma[i] = phi
                Jgamma[i,i-1] = Jphi[0]
                Jgamma[i,i] = Jphi[1]
        elif self.order == 2:
            for i in range(1, n-1):
                phi, Jphi = self.phi(x[i-1], x[i], x[i+1])
                gamma[i] = phi
                Jgamma[i,i-1] = Jphi[0]
                Jgamma[i,i] = Jphi[1]
                Jgamma[i,i+1] = Jphi[2]

        return gamma, Jgamma

    def phi(self, x):
        pass
    def phi(self, w, x):
        pass
    def phi(self, w, x, y):
        pass

class TargetVelocity(TaskMap):
    def __init__(self):
        super(TargetVelocity, self).__init__(name="target_velocity", order=1)
    def phi(self, w, x):
        v_cost = (x-w) - v_desired
        Jv_cost = np.array([-1, 1])
        return v_cost, Jv_cost

class AccelerationPenalty(TaskMap):
    def __init__(self):
        super(AccelerationPenalty, self).__init__(name="acceleration_penalty", order=2)
    def phi(self, w, x, y):
        a_cost = w-2*x+y
        Ja_cost = np.array([1, -2, 1])
        return a_cost, Ja_cost

class PyKOMO:
    def __init__(self):
        self.task_maps = []
        self.lambda_0 = 0.1
        self.rho = 0.01
        self.eps = 0.01

    def add_task(self, task):
        self.task_maps.append(task)

    def gamma(self, x):
        #dim = len(self.task_maps)

        gamma = np.zeros(x.shape[0])
        Jgamma = np.zeros((x.shape[0], x.shape[0]))

        for t in self.task_maps:
            _gamma, _Jgamma = t.gamma(x)
            gamma += _gamma
            Jgamma += _Jgamma

        return gamma, Jgamma

    def traj_cost(self, x):
        c, _ = self.gamma(x)
        return np.dot(c, c)

    def run(self, x0):
        _lambda = self.lambda_0
        _alpha = 1

        x = x0
        I = np.identity(x.shape[0])
        while True:
            gamma, Jgamma = self.gamma(x)
            JgammaT = np.transpose(Jgamma)
            A = 2 * np.matmul(JgammaT, Jgamma) + _lambda * I
            B = 2 * np.matmul(JgammaT, gamma)
            d = np.linalg.solve(A, -B)
            while self.traj_cost(x + _alpha * d) > self.traj_cost(x) + self.rho * np.matmul(np.transpose(B),
                                                                             _alpha * d):  # line search
                _alpha = _alpha * 0.5
            x = x + _alpha * d
            _alpha = 1

            if np.linalg.norm(_alpha * d) < self.eps:
                break

        return x

if __name__ == "__main__":
    #n = 10
    #x = np.zeros((2, 10))
    #x = np.array([0, 1, 2, 5, 4, 5, 6, 7])
    x0 = np.array([0, 0, 0, 1, 0, 0, -1, 0])
    #x = np.array([pow(x, 2) for x in range(0,10)])
    #path0 = range(0, len(x0))
    print("x0:{}".format(x0))
    #path0 = list(itertools.chain(range(0, 3), range(len(x0)-1, 2, -1)))
    #path0 = list(range(0, len(x0)))
    #print(list(path0))

    acc_phi = AccelerationPenalty()
    target_vel = TargetVelocity()

    komo = PyKOMO()
    #komo.add_task(acc_phi)
    komo.add_task(target_vel)
    x = komo.run(x0)

    print("final x:{}\ncost(x):{}".format(x, komo.traj_cost(x)))

