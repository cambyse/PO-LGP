from decorator import decorator

def dynamic_programming(f):
    def dp(f, *args, **kwargs):
        try:
            f.cache[args]
        except KeyError:
            f.cache[args] = f(*args, **kwargs)
        return f.cache[args]
    f.cache = {}
    f.clear = f.cache.clear
    return decorator(dp, f)
