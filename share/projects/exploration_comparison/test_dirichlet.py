import dirichlet


def test_uniform():
    dom = dirichlet.get_dom(3)

    # The likelihod at each loc is 1 in the uniform dirichlet distribution.
    # alphas = [1,1,1] results in the uniform distribution.
    alphas = 1, 1, 1
    for loc in dom:
        likelihood = dirichlet.pdf(loc, alphas)
        assert likelihood == 1.


def test_non_uniform():
    dom = dirichlet.get_dom(3)
    # the likelihood should not be one (but could at some loc)
    # TODO this is not a good unittest!
    alphas = 1, 5, 1
    for loc in dom:
        likelihood = dirichlet.pdf(loc, alphas)
        assert likelihood != 1.
