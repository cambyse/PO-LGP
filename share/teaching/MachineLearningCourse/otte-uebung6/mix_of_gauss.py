from pylab import *
from matplotlib.patches import Ellipse


def plotEllipse(pos, P, edge='black', face='0.3'):
    """Fancy plot for covariance matrix"""
    U, s , Vh = svd(P)
    orient = math.atan2(U[1,0],U[0,0])*180/pi
    ellipsePlot = Ellipse(
            xy = pos,
            width = 2.0 * math.sqrt(s[0]),
            height = 2.0 * math.sqrt(s[1]),
            fill = False,
            angle = orient)
    ax = gca()
    ax.add_patch(ellipsePlot);
    return ellipsePlot;


# init
data = loadtxt('mixture.txt')

max_iter = 20
K = 3
N = data.shape[0]
DIM = data.shape[1]

# pick random mu
mu = zeros((3, 2))
for i in range(K):
    mu[i,:] = data[randint(N),:]

# initialize Covariance matrix
tmp_sigma = eye(DIM)
Sigmas = [tmp_sigma, tmp_sigma, tmp_sigma]

q = zeros((K, N))
w_ki = zeros((K, N))

# iterate
for iteration in range(max_iter):
    print '=' * 60
    print 'iteration %d' % (iteration)

    #####################################
    #         Expectation
    #####################################
    for k in range(K):
        #print 'iteration %d, k %d' % (iteration, k)
        faktor = 1.0 / ((2 * pi)**(K/2.0) * det(Sigmas[k])**(1/2.0))
        for i in range(N):
            xminmu = data[i,:] - mu[k,:].T
            potenz = dot(dot((-1/2.0) * xminmu.T, Sigmas[k]), xminmu)
            res = faktor * exp(potenz)
            q[k,i] = res

    #normalize q
    for i in range(N):
        q[:, i] = q[:, i] / sum(q[:, i])
    print 'q[0] normalized', q[:,0], 'sum', sum(q[:,0])
    # make sure it's normalized
    assert sum(q[:,0]) + 0.01 > 1

    #####################################
    #         Maximize
    #####################################
    # gewichte
    print 'w_ki', w_ki.shape
    for k in range(K):
        w_ki[k,:] = q[k,:] / sum(q[k,:])
    # make sure it's normalized
    assert 1.0 - sum(w_ki[0,:]) < 000.1

    # schwerpunkte upaten
    mu = dot(w_ki, data)
    print 'schwerpunkte mu'
    print mu

    # covarianzmatrix updaten
    for k in range(K):
        Sigmas[k] = dot(dot(data.T, diag(w_ki[k,:])), data) - dot(mu[k:k+1,:].T, mu[k:k+1,:])
    print 'sigma[0]'
    print Sigmas[0]

    #for k in range(K):
        #plotEllipse(mu[k,:], Sigmas[k])
    #draw()

# some plotting
scatter(data[:,0], data[:,1])
grid()
for i in range(K):
    plotEllipse(mu[i,:], Sigmas[i])
draw()
show()
