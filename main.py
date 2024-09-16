# imports
import json
import time
import opengen as og
import casadi.casadi as cs
import matplotlib.pyplot as plt
import numpy as np
import sys
import statistics
import os
import configparser

# project modules
from RailwayClasses.Receiver import Receiver
from RailwayClasses.Train import Train
from RailwayClasses.ATO import ATO
from RailwayClasses.Transmitter import Transmitter
from RailwayClasses.CommNetwork import CommNetwork

def init_config():
    
    global path_variables, path_txt
    # Create a ConfigParser object
    config = configparser.ConfigParser()

    # Read the .ini file
    config.read('Resources/init/config.ini')
    
    # Accessing sections and values
    path_variables = config['path_files']['path_variables']
    path_txt = config['path_files']['path_txt']

# Load json file and its simulation variables
def load_data():
        
    # Load JSON file
    with open(path_variables, 'r') as file:
        data = json.load(file)
        
    global nu, nx, N_f, N_l, ts, ato_leader_ts, lambda_exp, min_delay_time
    global p_channel, M, A, B, C, Tf, delta_param, pos_leader, vel_leader, acc_leader
    global pos_follower, vel_follower, acc_follower, v_l_target, packet48
    global emergency_braking, d_vc, time_simulation, ref_tau_1, ref_tau_2, ref_tau_3
    global os1, os2, save_txt, plot
    global time_loss, duration_loss, flag_loss

    # Assign variables
    # ATO parameters
    nu = data['ATO_parameters']['nu']
    nx = data['ATO_parameters']['nx']
    N_f = data['ATO_parameters']['N_f']
    N_l = data['ATO_parameters']['N_l']
    ts = data['ATO_parameters']['ts']
    ato_leader_ts = data['ATO_parameters']['ato_leader_ts']

    # Communication channel parameters
    lambda_exp = data['Communication_channel_parameters']['lambda_exp']
    min_delay_time = data['Communication_channel_parameters']['min_delay_time']
    p_channel = data['Communication_channel_parameters']['p_channel']
    
    time_loss = data['Communication_channel_parameters']['time_loss']
    duration_loss = data['Communication_channel_parameters']['duration_loss']
    flag_loss = data['Communication_channel_parameters']['flag_loss']
  

    # Train parameters
    M = data['train_parameters']['M']
    A = data['train_parameters']['A']
    B = data['train_parameters']['B']
    C = data['train_parameters']['C']
    Tf = data['train_parameters']['Tf']
    delta_param = data['train_parameters']['delta_param']
    pos_leader = data['train_parameters']['pos_leader']
    vel_leader = data['train_parameters']['vel_leader']
    acc_leader = data['train_parameters']['acc_leader']
    pos_follower = data['train_parameters']['pos_follower']
    vel_follower = data['train_parameters']['vel_follower']
    acc_follower = data['train_parameters']['acc_follower']

    # Velocity target for the leader train
    v_l_target = data['velocity_target']['v_l_target']
    packet48 = data['velocity_target']['packet48']

    # Other parameters
    emergency_braking = data['emergency_braking']
    d_vc = data['d_vc']
    time_simulation = data['time_simulation']
    ref_tau_1 = data['ref_tau_1']
    ref_tau_2 = data['ref_tau_2']
    ref_tau_3 = data['ref_tau_3']
    
    os1 = data['os1']
    os2 = data['os2']
    
    save_txt = data['simulation_parameters']['save_txt']
    plot =  data['simulation_parameters']['plot']
# create the list where to store all the variables
def create_list():
    global s_f_list, s_l_list, v_f_list, v_l_list, a_f_list, a_l_list, u_f_list, u_l_list, interdistance
    global j_f_list, cost_f_list, cost_l_list, ref_f_list, ref_l_list, exeTime_f_list, exeTime_l_list
    global error_f_list, time_list, rlp_s_list, rlp_v_list, b_list, c_list, flag_eme_list, d_list,b_0_list
    global time_delay_list, delay_channel_list, z_tau_1_list, z_tau_2_list, z_tau_3_list, z_region_list, err_z_tau
    global event_b
    
    # Inizializzo tutte le liste come vuote
    s_f_list = list()
    s_l_list = list()
    v_f_list = list()
    v_l_list = list()
    a_f_list = list()
    a_l_list = list()
    u_f_list = list()
    u_l_list = list()
    j_f_list = list()
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
    z_region_list = list()
    err_z_tau = list()
    d_list = list()
    b_0_list = list()
    interdistance = list()
    event_b = list()

def init_simulation():
    
    global ato_follower, ato_leader, tx_leader, rx_follower
    global leader, follower, commNetwork
    global s_f, v_f, a_f, s_l, v_l, a_l
    
    ato_follower = ATO(N_f, ts,ref_tau_1, ref_tau_2, ref_tau_3)
    ato_leader = ATO(N_l, ato_leader_ts, s_1=None, s_2=None, s_3= None)
    tx_leader = Transmitter(None, ts, packet48)
    rx_follower = Receiver()

    # Setto condiizoni iniziali e parametri ai due treni
    leader = Train(pos_leader, vel_leader, acc_leader, ts, packet48, ato_leader, tx_leader, None)
    follower = Train(pos_follower, vel_follower, acc_follower, ts, None, ato_follower, None, rx_follower)
    commNetwork = CommNetwork(lambda_exp,min_delay_time, time_loss, duration_loss, flag_loss)
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
    
def plot_simulation():
    fig, axs = plt.subplots(2, 4)
    axs[0, 0].plot(time_list, error_f_list)
    #axs[0, 0].plot(time_list, err_z_tau)
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
    axs[1, 1].plot(time_list, j_f_list, 'tab:cyan')
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
    axs[0, 3].plot(time_delay_list, event_b, 'tab:green', marker='x',  linestyle='')
    axs[0, 3].set_title('b')
    axs[0, 3].grid()

    axs[1, 3].plot(time_delay_list, delay_channel_list, 'tab:red')
    axs[1, 3].set_title('delay communication channel')
    axs[1, 3].grid()


    plt.show()
    
def run_simulation():
    
    global nu, nx, N_f, N_l, ts, ato_leader_ts, lambda_exp, min_delay_time
    global p_channel, M, A, B, C, Tf, delta_param, pos_leader, vel_leader, acc_leader
    global pos_follower, vel_follower, acc_follower, v_l_target, packet48
    global emergency_braking, d_vc, time_simulation, ref_tau_1, ref_tau_2, ref_tau_3

    
    start_time = time.time()

    for t in range(0, int(time_simulation/ts), 1):
        timestamp = round(t*ts,2)
        print("time: "+str(round(timestamp,2)))
        
        # Step for railway system
        s_l, v_l, a_l, u_l_control, result_l, message_l = leader.step_leader(timestamp, v_l_target)
        message_l_channel = commNetwork.step(timestamp, message_l)
        s_f, v_f, a_f, j_f, u_f_control, result_f, delay_channel, z_tau_3, z_tau_2, z_tau_1, z_region, ref_tau = follower.step_follower( timestamp, message_l_channel)
        rlp_s, rlp_v, b, c, flag_eme = follower.get_vc_variables()
        
        err_z_tau.append(ref_tau-s_f)
        z_tau_1_list.append(z_tau_1)
        z_tau_2_list.append(z_tau_2)
        z_tau_3_list.append(z_tau_3)
        z_region_list.append(z_region)
        d_list.append(d_vc)
        b_0_list.append(0)
        interdistance.append(s_l-s_f)
        
        if delay_channel is not None:
            event_b.append(b)
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
        j_f_list.append(j_f)
        # store NMPC data
        exeTime_f_list.append(result_f.solve_time_ms)
        exeTime_l_list.append(result_l.solve_time_ms)
        error_f_list.append(s_l-s_f)
        cost_f_list.append(result_f.cost)
        
        if os1:
            if t*ts>700:
                v_l_target = 70
            if t*ts>1300:
                
                v_l_target = 60
            if t*ts>2000:
                v_l_target = 50
            if t*ts>2500:
                
                v_l_target = 30
            if t*ts>3000:
                v_l_target = 0
        if os2 and  t*ts>2000:
            commNetwork.set_param_channel(1.5)
        
            

    # used to clean the terminal
    os.system('cls' if os.name == 'nt' else 'clear')

    print("###################################")
    print("SOLVER EXECTUION PARAMETERS")

    print("FOLLOWER - mean: "+ str(statistics.mean(exeTime_f_list))+ "var: "+ str(statistics.variance(exeTime_f_list))  + "  maxTime: " + str(max(exeTime_f_list)))
    print("LEADER - mean: "+ str(statistics.mean(exeTime_l_list))+ "var: "+ str(statistics.variance(exeTime_l_list))  + "  maxTime: " + str(max(exeTime_l_list)))
    print("###################################")
    print("Execution time: %s seconds " % (time.time() - start_time))
    
    
def create_txt_from_lists(list1, list2, N, filename):
    # Ensure both lists have the same length
    if len(list1) != len(list2):
        raise ValueError("Both lists must have the same length.")
    
    # subsampling the lists
    list1 = [round(val, 2) for val in list1[::N]]
    list2 = [round(val, 2) for val in list2[::N]]
    
    folder_path = path_txt
    
    # Ensure the folder exists; create it if not
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
    
    # Join folder path and filename to get the full path
    file_path = os.path.join(folder_path, filename)
    
    # Open a file to write in text format
    with open(file_path, 'w') as file:
        for x, y in zip(list1, list2):
            file.write(f"{x} {y}\n")
    
    

def save_txt_files():
    
    #subsampling the vector
    N = 10
    
    create_txt_from_lists(time_list,a_f_list,N,"accelerationFollower.txt")
    create_txt_from_lists(time_list,a_l_list,N,"accelerationLeader.txt")
    create_txt_from_lists(time_list,b_list,N,"B.txt")
    create_txt_from_lists(time_list,s_f_list,N,"distanceFollower.txt")
    create_txt_from_lists(time_list,s_l_list,N,"distanceLeader.txt")
    create_txt_from_lists(time_list,v_f_list,N,"velocityFollower.txt")
    create_txt_from_lists(time_list,v_l_list,N,"velocityLeader.txt")
    create_txt_from_lists(time_list,exeTime_f_list,N,"exeController.txt")
    create_txt_from_lists(time_list,u_f_list,N,"forceFollower.txt")
    create_txt_from_lists(time_list,u_l_list,N,"forceLeader.txt")
    create_txt_from_lists(time_list,z_tau_1_list,N,"z_tau_1.txt")
    create_txt_from_lists(time_list,z_tau_2_list,N,"z_tau_2.txt")
    create_txt_from_lists(time_list,z_tau_3_list,N,"z_tau_3.txt")
    create_txt_from_lists(time_list,z_region_list,N,"Z_tau.txt")
    create_txt_from_lists(time_list,interdistance,N,"interDistance.txt")
    create_txt_from_lists(time_delay_list,delay_channel_list,N,"tauEstimated.txt")
    create_txt_from_lists(time_list,d_list,N,"d.txt")
    create_txt_from_lists(time_list,b_0_list,N,"b_0.txt")
    
init_config()
load_data()
create_list()
init_simulation()
run_simulation()
if save_txt:
    save_txt_files()
if plot:
    plot_simulation()





    
