import opengen as og
import casadi.casadi as cs
import matplotlib.pyplot as plt
import numpy as np

# Build parametric optimizer
# ------------------------------------
(nu, nx, np, N, ts) = (1, 3, 200, 50, 0.24)

(qv, qs, r, r_past,qN) = (0.5, 0.1, 0.015, 0.3, 1)
(M,A,B,C,Tf) = (490000, 26.152, 8.365, 1.914, 1)

u = cs.SX.sym('u', nu*N)
z0 = cs.SX.sym('z0', nx+N)

(s, v, uPast) = (z0[0], z0[1], z0[2])

cost = 0
fu_P = []
fu_J = []
fv = []

u_t_past= uPast

for t in range(0, nu*(N-3), nu):
    
    u_t = u[t]
    cost += (qv*(v-z0[t+3]))**2 + (r*u_t)**2
    #cost += r * cs.dot(u_t, u_t) + r_past*cs.dot((u_t-u_t_past), (u_t-u_t_past))
    if t>0:
        cost +=  (r_past*(u[t]-u[t-1]))**2
    else:
        cost +=  (r_past*(u[t]-u_t_past))**2
    
    s += ts*v 
    v += ts*((1/M)*(-A-B*v-Tf*C*v**2)+u_t*1000/M)
    fu_P =cs.vertcat(fu_P, u_t*v)
    fv =cs.vertcat(fv, v)
    #if t>0:
    #    fu_J = cs.vertcat(fu_J,cs.fmax(0.0, cs.fabs(u[t] - u[t-1]) - 10))
    
    if t>0:
        fu_J = cs.vertcat(fu_J,cs.fmax(0.0, cs.fabs(u[t] - u[t-1]) - 1))
        
    else:
        fu_J = cs.vertcat(fu_J,cs.fmax(0.0, cs.fabs(u[t] - u_t_past) - 1))
        
        
        #fu_J =cs.vertcat(fu_J, ((u[t] - u[t-1])))
    u_t_past = u_t

cost += (qv*(v-z0[t+3]))**2 + (r*u_t)**2 + (r_past*(u[t]-u[t-1]))**2

umin = [-300.0] * (nu*N)
umax = [370.0] * (nu*N)
bounds = og.constraints.Rectangle(umin, umax)

ftot = []
ftot = cs.vertcat(ftot, fu_P)
ftot = cs.vertcat(ftot, fu_J)
ftot = cs.vertcat(ftot, fv)

fmin = [-30710]*(fu_P.size1()) + [0]*(fv.size1()) #+ [-3]*(fu_J.size1()) 
fmax = [37000]*(fu_P.size1()) + [83.3]*(fv.size1())#+ [3]*(fu_J.size1()) 

# fmin = [0]*(fu_P.size1()) + [0]*(fv.size1()) 
# fmax = [30710]*(fu_P.size1()) +[83.3]*(fv.size1())

set_c = og.constraints.Rectangle(fmin, fmax)
set_y = og.constraints.BallInf(None, 1e12)


#    .with_penalty_constraints(fu_J)        \

problem = og.builder.Problem(u, z0, cost)\
    .with_constraints(bounds)  
build_config = og.config.BuildConfiguration()\
    .with_build_directory("Controller")\
    .with_open_version("0.8.1")\
    .with_build_mode("release")\
    .with_build_python_bindings()\
    .with_rebuild(True)
meta = og.config.OptimizerMeta()\
    .with_optimizer_name("controller_solver")
solver_config = og.config.SolverConfiguration()\
    .with_tolerance(1e-4) 
builder = og.builder.OpEnOptimizerBuilder(problem,
                                          meta,
                                          build_config,
                                          solver_config)
builder.build()

