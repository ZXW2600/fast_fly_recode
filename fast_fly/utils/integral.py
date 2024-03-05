import casadi as ca
def RK4(f_c:ca.Function, X0, U, dt, M:int):
    DT = dt/M
    X1 = X0
    for _ in range(M):
        k1 = DT*f_c(X1,        U)
        k2 = DT*f_c(X1+0.5*k1, U)
        k3 = DT*f_c(X1+0.5*k2, U)
        k4 = DT*f_c(X1+k3,     U)
        X1 = X1+(k1+2*k2+2*k3+k4)/6
    # F = ca.Function('F', [X0, U], [X1] ,['X0', 'U'], ['X1'])
    return X1

def EulerIntegral(f_c:ca.Function, X0, U, dt, M:int):
    DT = dt/M
    X1 = X0
    for _ in range(M):
        X1 = X1 + DT*f_c(X1, U)
    
    return X1