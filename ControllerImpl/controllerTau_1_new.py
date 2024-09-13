import opengen as og
import casadi.casadi as cs
import matplotlib.pyplot as plt
import numpy as np

# Build parametric optimizer
# ------------------------------------
(nu, nx, N, ts) = (1, 4, 17, 0.12)

(qv, qs, r, r_past) = (0.2, 0.2, 0.35, 200)
(M,A,B,C,Tf) = (490000, 26.152, 8.365, 1.914, 0.5)

u = cs.SX.sym('u', nu*N)
z0 = cs.SX.sym('z0', nx+N)

(s, v, v_target, uPast) = (z0[0], z0[1], z0[2], z0[3])

cost = 0
fu_P = []
fu_J = []
fv = []



u_t_past= uPast

for t in range(0, nu*N-1, nu):
    print(t+2)

    u_t = u[t]
     
    v += ts*((1/M)*(-A-B*v-Tf*C*v**2)+u_t*10000/M)
    s += ts*v
    fv =cs.vertcat(fv, v)
    
    cost += (qs*(s-z0[t+4]))**2 + (r*u_t)**2 + (qv*(v-v_target))**2 + r_past*cs.dot((u_t-u_t_past), (u_t-u_t_past))
    
    if t==0:
        fu_J = cs.vertcat(fu_J,cs.fmax(0.0, cs.fabs(u[t] - u_t_past) - 0.003))


    
    u_t_past = u_t
    fv = cs.vertcat(fv, v)
    #fu_J = cs.vertcat(fu_J,cs.fmax(0.0, cs.fabs(s-z0[t+3]) ))

#cost += ((qv*(v-z0[t+2]))**2) 

umin = [-37.0] * (nu*N)
umax = [37.0] * (nu*N)
bounds = og.constraints.Rectangle(umin, umax)

fmin = [0]*(fv.size1()) 
fmax =  [83]*(fv.size1()) 

set_c = og.constraints.Rectangle(fmin, fmax)
set_y = og.constraints.BallInf(None, 1e12)

ftot = []

ftot = cs.vertcat(ftot, fv)


#    .with_penalty_constraints(fu_J)        \

problem = og.builder.Problem(u, z0, cost)\
    .with_penalty_constraints(fu_J)        \
    .with_constraints(bounds)   \
    .with_aug_lagrangian_constraints(ftot, set_c, set_y) 
build_config = og.config.BuildConfiguration()\
    .with_build_directory("Controller")\
    .with_open_version("0.8.1")\
    .with_build_mode("release")\
    .with_build_python_bindings()
meta = og.config.OptimizerMeta()\
    .with_optimizer_name("controller_tau_1_new")
solver_config = og.config.SolverConfiguration()  \
    .with_tolerance(1e-5)  \
    .with_initial_tolerance(1e-5) \
    .with_lbfgs_memory(20) \
    .with_delta_tolerance(1e-1) \
    .with_penalty_weight_update_factor(8.0) \
    .with_initial_penalty(100.0)
builder = og.builder.OpEnOptimizerBuilder(problem,
                                          meta,
                                          build_config,
                                          solver_config)
builder.build()

