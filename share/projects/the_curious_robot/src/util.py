import orspy as ors
class Gaussian():
    """
    Represent Gaussians with this class.
    TODO maybe there is a Gaussian class that we can recycle.
    """
    def __init__(self, mu=0, sigma=99999):
        self.mu = mu
        self.sigma = sigma


class Properties():
    """Potential properties a DoF can have"""
    def __init__(self):
        self.joint = Gaussian()
        self.friction = Gaussian()
        self.weight = Gaussian()
        self.limit_min = Gaussian()
        self.limit_max = Gaussian()

    def property_names(self):
        return [attr for attr in dir(self)
                if not (attr.startswith("__") or
                        callable(getattr(self, attr)))]


def parse_body_msg(msg):
    body = ors.Body()
    body_list = msg.split(' ', 1)
    body.name = body_list[0]
    body.read(body_list[1])
    return body
