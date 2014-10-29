from decorator import decorator

def _dynamic_programming(f, *args, **kwargs):
    try:
        f.cache[args]
    except KeyError:
        f.cache[args] = f(*args, **kwargs)
    return f.cache[args]

def dynamic_programming(f):
    f.cache = {}
    def ec():
        f.cache.clear()
    f.empty_cache = ec
    return decorator(_dynamic_programming, f)
