
def flatten(xss):
    if not isinstance(xss, list):
        return [xss]
    if len(xss) == 0:
        return []
    if not isinstance(xss[0], list):
        return xss
    return [x for xs in xss for x in xs]
