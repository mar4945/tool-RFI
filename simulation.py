# imports
import time
import opengen as og
import casadi.casadi as cs
import matplotlib.pyplot as plt
import numpy as np
import sys
import statistics
import os

# project modules
from Receiver import Receiver
from Train import Train
from ATO import ATO
from Transmitter import Transmitter
from CommNetwork import CommNetwork

# TODO create json file to configure parameters

# ATO parameters
(nu, nx, N_f, N_l, ts, ato_leader_ts) = (1, 2, 34, 200, 0.06, 0.06)
# Communication channel parameters
(lambda_exp, min_delay_time, p_channel) = (0.1, 0.8, 0.8)

# train parameters
(M,A,B,C,Tf, delta_param) = (490000, 26.152, 8.365, 1.914, 1, 0.1)
(pos_leader, vel_leader, acc_leader) = (6000, 50, 0)
(pos_follower, vel_follower, acc_follower) = (0, 50, 0)

# velocity target for the leader train
(v_l_target, packet48) = (50, 1.02)

emergency_braking = -370000
d_vc = 1000

# tempo di simulazione
time_simulation = 600


# list used to store simulation data
s_f_list = list()
s_l_list = list()
v_f_list = list()
v_l_list = list()
a_f_list = list()
a_l_list = list()
u_f_list = list()
u_l_list = list()
cost_f_list = list()
cost_l_list = list()
ref_f_list = list()
ref_l_list = list()
exeTime_f_list = list()
exeTime_l_list = list()
error_f_list = list()
time_list = list()
rlp_s_list = list()
rlp_v_list = list()
b_list = list()
c_list = list()
flag_eme_list = list()
time_delay_list = list()
delay_channel_list = list()

z_tau_1_list = list()
z_tau_2_list = list()
z_tau_3_list = list()
z_region_list= list()
err_z_tau = list()

ref_tau_1 = pos_leader-1200
ref_tau_2 = pos_leader-1250
ref_tau_3 = pos_leader-1300

ato_follower = ATO(N_f, ts,ref_tau_1, ref_tau_2, ref_tau_3)
ato_leader = ATO(N_l, ato_leader_ts, s_1=None, s_2=None, s_3= None)
tx_leader = Transmitter(None, ts, packet48)
rx_follower = Receiver()

# Setto condiizoni iniziali e parametri ai due treni
leader = Train(pos_leader, vel_leader, acc_leader, ts, packet48, ato_leader, tx_leader, None)
follower = Train(pos_follower, vel_follower, acc_follower, ts, None, ato_follower, None, rx_follower)
commNetwork = CommNetwork(lambda_exp,min_delay_time)
follower.set_parameters(M, A, B, C, delta_param, emergency_braking, 
                        d_vc, leader.position,leader.velocity, min_delay_time, p_channel)
leader.set_parameters(M, A, B, C, delta_param, None, None,None, None, min_delay=None, p_channel=None)

# Condizioni iniziali leader e follower
s_f = follower.position
v_f = follower.velocity
a_f = follower.acceleration

s_l = leader.position
v_l = leader.velocity
a_l = leader.acceleration

start_time = time.time()

for t in range(0, int(time_simulation/ts), 1):
    timestamp = round(t*ts,2)
    print("time: "+str(round(timestamp,2)))

    # Step for railway system
    s_l, v_l, a_l, u_l_control, result_l, message_l = leader.step_leader(timestamp, v_l_target)
    message_l_channel = commNetwork.step(timestamp, message_l)
    s_f, v_f, a_f, u_f_control, result_f, delay_channel, z_tau_3, z_tau_2, z_tau_1, z_region, ref_tau = follower.step_follower( v_l, timestamp, message_l_channel)
    rlp_s, rlp_v, b, c, flag_eme = follower.get_vc_variables()
    
    err_z_tau.append(ref_tau-s_f)
    z_tau_1_list.append(z_tau_1)
    z_tau_2_list.append(z_tau_2)
    z_tau_3_list.append(z_tau_3)
    z_region_list.append(z_region)
    
    if delay_channel is not None:
        delay_channel_list.append(delay_channel)
        time_delay_list.append(timestamp)
    
    rlp_s_list.append(rlp_s)
    rlp_v_list.append(rlp_v)
    b_list.append(b)
    c_list.append(c)
    flag_eme_list.append(flag_eme)
    
    
    # store time simulation
    time_list.append(t*ts)
    # store control inputs
    u_f_list.append(u_f_control)
    u_l_list.append(u_l_control)    
    # store train data
    s_f_list.append(s_f)
    s_l_list.append(s_l)
    v_l_list.append(v_l)
    v_f_list.append(v_f)
    a_l_list.append(a_l)
    a_f_list.append(a_f)
    # store NMPC data
    exeTime_f_list.append(result_f.solve_time_ms)
    exeTime_l_list.append(result_l.solve_time_ms)
    error_f_list.append(s_l-s_f)
    cost_f_list.append(result_f.cost)
    
    if t*ts>2000:
        v_l_target = 0
        #commNetwork.set_param_channel(2)
    
print('\a')
# used to clean the terminal
os.system('cls' if os.name == 'nt' else 'clear')

print("###################################")
print("SOLVER EXECTUION PARAMETERS")
# print("-----------------------------------")
# print("exit_status: "+ str(result_f.exit_status))
# print("num_outer_iterations: " + str(result_f.num_outer_iterations))
# print("num_inner_iterations: " + str(result_f.num_inner_iterations))
# print("solve_time_ms: "+ str(result_f.solve_time_ms)) 
# print("last_problem_norm_fpr: "+ str(result_f.last_problem_norm_fpr))
# print("f1_infeasibility: "+ str(result_f.f1_infeasibility))
# print("f2_norm: "+ str(result_f.f2_norm))
# print("penalty: "+ str(result_f.penalty))
# print("cost: "+ str(result_f.cost))
# print("solution: "+ str(result_l.solution))
# print("-----------------------------------")
# print("###################################")
# print("max v: "+ str(max(v_f_list)))
# print("max u: "+ str(max(u_f_list)))
print("FOLLOWER - mean: "+ str(statistics.mean(exeTime_f_list))+ "var: "+ str(statistics.variance(exeTime_f_list))  + "  maxTime: " + str(max(exeTime_f_list)))
print("LEADER - mean: "+ str(statistics.mean(exeTime_l_list))+ "var: "+ str(statistics.variance(exeTime_l_list))  + "  maxTime: " + str(max(exeTime_l_list)))
print("###################################")
print("Execution time: %s seconds " % (time.time() - start_time))



fig, axs = plt.subplots(2, 4)
axs[0, 0].plot(time_list, error_f_list)
axs[0, 0].plot(time_list, err_z_tau)
axs[0, 0].set_title('error follower')
axs[0, 0].grid()


axs[0, 1].plot(time_list, s_l_list, 'tab:olive')
axs[0, 1].plot(time_list, z_tau_1_list, 'tab:red')
axs[0, 1].plot(time_list, z_tau_2_list, 'tab:orange')
axs[0, 1].plot(time_list, z_tau_3_list, 'tab:green')
axs[0, 1].plot(time_list, s_f_list, 'tab:cyan')
axs[0, 1].set_title('follower position')
axs[0, 1].grid()

axs[1, 0].plot(time_list, u_f_list, 'tab:red')
axs[1, 0].plot(time_list, u_l_list, 'tab:green')
axs[1, 0].set_title('input')
axs[1, 0].grid()

axs[1, 1].plot(time_list, a_f_list, 'tab:red')
axs[1, 1].plot(time_list, a_l_list, 'tab:green')
axs[1, 1].set_title('Accelerations')
axs[1, 1].grid()

axs[1, 2].plot(time_list, v_f_list, 'tab:red')
axs[1, 2].plot(time_list, v_l_list, 'tab:green')
axs[1, 2].set_title('velocity')
axs[1, 2].grid()

axs[0, 2].plot(time_list, z_region_list, 'tab:red')
#axs[0, 2].plot(time_list, exeTime_l_list, 'tab:green')
axs[0, 2].set_title('Z tau regions')
axs[0, 2].grid()

axs[0, 3].plot(time_list, b_list, 'tab:red')
axs[0, 3].set_title('b')
axs[0, 3].grid()

axs[1, 3].stem(time_delay_list, delay_channel_list, 'tab:red')
axs[1, 3].set_title('delay communication channel')
axs[1, 3].grid()


plt.show()




    
