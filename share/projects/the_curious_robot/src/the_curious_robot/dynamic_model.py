# -*- coding: utf-8 -*-

"""
Learning the Dynamics of a System
assume the dynamic system is a point mass which state is the position and
velocity. The velocity is decreased by a damping factor.

"""
from __future__ import division
import numpy as np
import matplotlib.pyplot as plt
import pymc as pm
import math


# Generative Model
def point_mass_integrate(p0, v0, damping, N, dt, min_limit, max_limit):
    positions = np.zeros((N,))
    velocities = np.zeros((N,))
    positions[0] = p0
    velocities[0] = v0
    for n in range(N - 1):
        # Euler integration
        positions[n + 1] = positions[n] + velocities[n] * dt
        direction = 1 if velocities[n] > 0 else -1
        if positions[n + 1] > max_limit or positions[n+1] < min_limit:
            direction = -direction
        tmp_sqr_vel = max(velocities[n]**2 -
                          abs(2 * damping * velocities[n] * dt), 0)
        velocities[n + 1] = direction * math.sqrt(tmp_sqr_vel)

    return (positions, velocities)


class DynamicModel:
    def __init__(self,
                 prior_damping=pm.Uniform("damping", lower=0., upper=10.),
                 prior_init_pos=pm.Uniform("init_pos", lower=-1.0, upper=1.0),
                 prior_init_vel=pm.Uniform("init_vel", lower=0.0, upper=2.0),
                 prior_min_limit=pm.Uniform("min_limit", lower=-1.5, upper=1.5),
                 prior_max_limit=pm.Uniform("max_limit", lower=-1.0, upper=2.0)):

        self.prior_damping = prior_damping

        self.prior_init_pos = prior_init_pos
        self.prior_init_vel = prior_init_vel

        self.prior_min_limit = prior_min_limit
        self.prior_max_limit = prior_max_limit

        # Standard deviation is modelled with a Uniform prior
        self.std_pos = pm.Uniform("std", lower=0.0, upper=.1)

    def add_observations(self, observations, dt, samples=10000, burnout=5000,
                         thinning=2):
        N = observations.shape[0]

        @pm.deterministic
        def est_pos(damping=self.prior_damping,
                    init_pos=self.prior_init_pos,
                    init_vel=self.prior_init_vel,
                    min_limit=self.prior_min_limit,
                    max_limit=self.prior_max_limit):
            """
            Estimate N positions of the point mass for given damping,
            init_pos, and init_vel.
            """
            pos, vel = point_mass_integrate(init_pos, init_vel, damping.item(),
                                            N, dt, min_limit, max_limit)
            return pos

        @pm.deterministic
        def est_pos_prec(std_pos=self.std_pos):
            """Transform std into precition"""
            return 1.0/std_pos**2

        # The observations are based on the estimated positions and precision
        obs_pos = pm.Normal("obs", mu=est_pos, tau=est_pos_prec,
                            value=observations, observed=True)
        self.mcmc = pm.MCMC([self.prior_damping,
                             self.prior_init_pos,
                             self.prior_init_vel,
                             self.prior_min_limit,
                             self.prior_max_limit,
                             est_pos_prec,
                             obs_pos])

        dist_map = pm.MAP(self.mcmc)
        dist_map.fit()

        self.mcmc.sample(samples, burnout, thinning)

        # update priors to posteriors
        self.prior_damping = self._get_approx_gaussian("damping")
        self.prior_init_pos = self._get_approx_gaussian("init_pos")
        self.prior_init_vel = self._get_approx_gaussian("init_vel")
        self.prior_min_limit = self._get_approx_gaussian("min_limit")
        self.prior_max_limit = self._get_approx_gaussian("max_limit")

    def _get_approx_gaussian(self, name):
        samples = self.mcmc.trace(name)[:]
        est_mean = np.mean(samples)
        est_std = np.std(samples)
        est_tau = 1. / (est_std * est_std)
        return pm.Normal(name, mu=est_mean, tau=est_tau)

    def get_approx_gaussian(self, name):
        samples = self.mcmc.trace(name)[:]
        est_mean = np.mean(samples)
        est_std = np.std(samples)
        return (est_mean, est_std)
