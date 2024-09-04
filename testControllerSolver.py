# Note: to use the direct interface you need to build using
#       .with_build_pythong_bindings()
import opengen as og
import casadi.casadi as cs
import matplotlib.pyplot as plt
import numpy as np
import sys
import statistics
import os




# Ottiene il percorso del file corrente
path_controller = os.path.dirname(os.path.abspath(__file__))
print(path_controller)

sys.path.insert(1, path_controller+"/Controller/controller_solver")
from Train import Train
import controller_solver # type: ignore

def MPCBezierController(P,u_guess):
    
    solver = controller_solver.solver()
    result = solver.run(p=P,
                        initial_guess=u_guess)
    
    if result is None:
        print("ciao")
    u_star = result.solution
    return u_star[0:nu*N:nu], result
      





time = list()
sl_list = list()
vf_list = list()
vl_list = list()
u_vect = list()
ref_vect = list()








# Parametri dei treni
(nu, nx, N, ts) = (1, 2, 100, 0.12)

ts = 0.03
(M,A,B,C,Tf) = (490000, 26.152, 8.365, 1.914, 1)
(pos_leader, vel_leader, acc_leader) = (6000, 40, 0)
(pos_follower, vel_follower, acc_follower) = (0, 40, 0)

# Setto condiizoni iniziali e parametri ai due treni
follower = Train(pos_follower, vel_follower, acc_follower, ts)
leader = Train(pos_leader, vel_leader, acc_leader, ts)
follower.set_parameters(M, A, B, C)
leader.set_parameters(M, A, B, C)




sf = list()
sl = list()
vf = list()
vl = list()
af = list()
al = list()
diff_s = list()
diff_v = list()
timeRef = list()
cost_vect = list()






time = list()
u_t = 5
a_vect = list()
e_vect = list()
exeTime_vect = list()
u_guess = [0]*(nu*N)

u_past = 10

# Condizioni iniziali leader e follower
s_f = follower.position
v_f = follower.velocity
a_f = follower.acceleration


s_l = leader.position
v_l = leader.velocity
a_l = leader.acceleration

v_target = 60

# tempo di simulazione
time_simulation = 200

for t in range(0, int(time_simulation/ts), 1):
    print(t*ts)
    time.append(t*ts)
    
    ref = [v_target for i in range(0, N)]
    
    #print(ref)
    #
    P = [s_l]+[v_l] +  [u_past] + ref
    #print(P)
    ref_vect.append(ref[0])
    
    [uMPC,result] = MPCBezierController(P,u_guess)
    
    u_guess = uMPC
    
    if str(result.exit_status)!="Converged":
        #print("exit_status: "+ str(result.exit_status) + "time: " + str(t*ts))
        a=1
    
    #print("cost: "+ str(result.cost))
    #print("solve_time_ms: "+ str(result.solve_time_ms)) 
    
    exeTime_vect.append(result.solve_time_ms)
    u_t = uMPC[0]
    u_past = u_t
    #print(result.solution)
    u_vect.append(u_t)
    
    
    
    sl_list.append(s_l)
    
    s_l, v_l, a_l= leader.dynamic(u_t*1000)
    a_vect.append(a_l)
    
    #print("error: " +str(sld-s))
    #tt.sleep(0.1)
    e_vect.append(v_target-v_l)

    

    
    cost_vect.append(result.cost)
    vl_list.append(v_l)
    
    #print(Vref[0])
    
    
    

print("###################################")
print("SOLVER EXECTUION PARAMETERS")
print("-----------------------------------")
print("exit_status: "+ str(result.exit_status))
print("num_outer_iterations: " + str(result.num_outer_iterations))
print("num_inner_iterations: " + str(result.num_inner_iterations))
print("solve_time_ms: "+ str(result.solve_time_ms)) 
print("last_problem_norm_fpr: "+ str(result.last_problem_norm_fpr))
print("f1_infeasibility: "+ str(result.f1_infeasibility))
print("f2_norm: "+ str(result.f2_norm))
print("penalty: "+ str(result.penalty))
print("cost: "+ str(result.cost))
#print("solution: "+ str(result.solution))
print("-----------------------------------")
print("###################################")
print("max u: "+ str(max(u_vect)))
print("mean: "+ str(statistics.mean(exeTime_vect))+ "var: "+ str(statistics.variance(exeTime_vect))  + "  maxTime: " + str(max(exeTime_vect)))


fig, axs = plt.subplots(2, 3)
axs[0, 0].plot(time, e_vect)
axs[0, 0].set_title('error')
axs[0, 0].grid()


axs[0, 1].plot(time, ref_vect, 'tab:green')
axs[0, 1].plot(time, sl_list, 'tab:red')
axs[0, 1].set_title('velocity')
axs[0, 1].grid()

axs[1, 0].plot(time, u_vect, 'tab:red')
axs[1, 0].set_title('input')
axs[1, 0].grid()

axs[1, 1].plot(time, a_vect, 'tab:red')
axs[1, 1].set_title('Acceleration')
axs[1, 1].grid()


axs[1, 2].plot(time, vl_list, 'tab:red')
axs[1, 2].set_title('velocity')
axs[1, 2].grid()

axs[0, 2].plot(time, cost_vect, 'tab:red')
axs[0, 2].set_title('cost_function')
axs[0, 2].grid()


plt.show()




    
