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




# msg parser/builder

def parse_body_msg(msg):
    body = ors.Body()
    body_list = msg.split(' ', 1)
    body.name = body_list[0]
    body.read(body_list[1])
    return body

def parse_property_msg(msg):
    properties = Properties()
    for p in msg:
        setattr(properties, p.name, Gaussian(p.values[0], p.values[1]))
    return properties

def parse_ooi_msg(msg):
    body = parse_body_msg(msg.body)
    properties = parse_property_msg(msg.properties)
    return {"body": body, "properties": properties}

def parse_oois_msg(msg):
    objects = []
    for obj in msg.objects:
        objects.append(parse_ooi_msg(obj))
    return objects


