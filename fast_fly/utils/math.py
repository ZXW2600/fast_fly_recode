
def constrain(a, lb, ub):
    if a<lb:
        a=lb
    if a>ub:
        a=ub
    return a