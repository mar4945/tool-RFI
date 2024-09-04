import numpy as np
from Transmitter import Transmitter
from Message import Message
import time
import math


class Train:
    def __init__(self, position=0.0, velocity=0.0, acceleration=0.0, ts=0.0, 
                 packet48 = 1.02, ato = None, transmitter= None, receiver = None):
        """
        Inizializza un'istanza della classe Train.

        :param position: La posizione del treno (es. in metri)
        :param velocity: La velocità del treno (es. in metri al secondo)
        :param acceleration: L'accelerazione del treno (es. in metri al secondo quadrato)
        :param ts: Timestamp o tempo
        """
        self.position = position
        self.velocity = velocity
        self.acceleration = acceleration
        self.ts = ts
        self.packet48 = packet48
        
        # ATO MODULE
        self.ato = ato
        # TRASNMITTER AND RECEIVER MODULES, THESE ARE optional depending on leader or follower train
        self.transmitter = transmitter
        self.receiver = receiver
        
        self.control = 0.0  # Valore di controllo iniziale

        # Parametri per il nuovo modello dinamico
        self.A = None
        self.B = None
        self.C = None
        self.M = None
        self.em_braking = None
        self.d_vc = None
        
        self.rho = 10**6
        self.slope = 0.0
        self.g = 9.81
        self.gamma = 10**6
        self.sigma = 0
        
        self.rlp_s = None
        self.rlp_v = None
        self.b = None
        self.c = None
        self.flag_emergency = False
        # window stack to estimate the lambda factor for the exponential distribution
        self.stack_window_channel = list()
        self.len_stack = 20
        self.p_channel = None
        self.min_T = None
        
        self.tau_bar = 1

    def set_parameters(self, M, A, B, C, delta_param, emergency_braking, d_vc, rlp_s, rlp_v, min_delay,p_channel):
        """
        Imposta i parametri del treno per il modello dinamico.

        :param A: Coefficiente A per la resistenza
        :param B: Coefficiente B per la resistenza
        :param C: Coefficiente C per la resistenza
        """
        self.A = A
        self.B = B
        self.C = C
        self.M = M
        self.em_braking = emergency_braking
        self.d_vc = d_vc
        self.rlp_s = rlp_s
        self.rlp_v = rlp_v
        self.p_channel = p_channel
        self.min_T = min_delay
        
        # Robust parameter
        self.M_min = round(M - M*delta_param,2)
        self.M_max = round(M + M*delta_param,2)

        self.A_min = round(A - A*delta_param,2)
        self.A_max = round(A + A*delta_param,2)

        self.B_min = round(B - B*delta_param,2)
        self.B_max = round(B + B*delta_param,2)

        self.C_min = round(C - C*delta_param,2)
        self.C_max = round(C + C*delta_param,2)
        
    def dynamic(self, control):
        """
        Aggiorna lo stato dinamico del treno in base al controllo e al tempo.

        :param control: Input di controllo che influenza l'accelerazione
        :param delta_time: Intervallo di tempo (es. in secondi)
        """
        # Forze agenti sul treno
        F_g = self.g * self.slope  # Forza gravitazionale
        F_r = 6 * self.gamma / self.rho  # Forza resistente
        F_e = F_g + F_r  # Forza totale

        # Resistenza aerodinamica e da attrito in base alla velocità
        R = self.A + self.velocity * self.B + self.C * self.velocity**2

        
        # Calcolo dell'accelerazione in base al controllo e alla resistenza
        self.acceleration = (1/self.M)*(control - R - F_e) 
        
        if self.velocity<=0:
            control = 0
            self.acceleration = 0

        # Aggiorna la velocità in base all'accelerazione
        self.velocity += self.acceleration * self.ts

        # Aggiorna la posizione in base alla velocità
        self.position += self.velocity * self.ts
        
        return self.position, self.velocity, self.acceleration
    
    def step_follower(self,  v_l, timestamp, message_l_channel):
        
        # update the leader message stored in the receiver
        delay_channel = self.receiver.step(message_l_channel)
        tau_estimated = None
        if delay_channel is not None:
            tau_bar = self.mle_estimator(delay_channel)
            self.tau_bar = tau_bar
            tau_estimated = self.tau_bar
            z_F_1, time_ref_1 =self.compute_Z_F(timestamp,1*self.tau_bar)
            z_F_2, time_ref_2 =self.compute_Z_F(timestamp,2*self.tau_bar)
            z_F_3, time_ref_3 =self.compute_Z_F(timestamp,3*self.tau_bar)
            
            self.ato.set_z_tau_ref(z_F_1, time_ref_1, z_F_2, time_ref_2, z_F_3, time_ref_3 )
            
        
        # get the last message sent from leader
        last_message_leader = self.receiver.last_message
        # compute the rlp prediction through the block
        self.leader_rlp_predictor_block(last_message_leader)
        
        # compute the flag for the emergency case through the block
        flag_emergency = self.safety_control_block(timestamp)
        
        u_emergency = self.ato.emergency_controller()
        
                       
        # calculate the control input using the NMPC
        u_f_control, result_f, z_tau_3, z_tau_2, z_tau_1, z_region, ref_tau  = self.ato.cruise_virtual_coupling(self.position, 
                                                                 self.velocity, v_l, timestamp, last_message_leader)
        
        
        if not flag_emergency:
            # take the first element for the actual time from VC controller
            u_0 = u_f_control[0]
            self.ato.set_emergency_braking(u_0)
        else:
            # if an emergency occurs, use the emergency controller
            u_0 = u_emergency
            #print(u_0)
            self.ato.set_emergency_braking(u_0)
            self.ato.set_u_past(u_0/self.ato.U_FACTOR_VC)
            z_region = 4
            
        if self.velocity <= 0:
            u_0 = 0
        
        
        
        # run the model dynamic
        s_f, v_f, a_f = self.dynamic(u_0)
        
        return s_f, v_f, a_f, u_0, result_f, tau_estimated, z_tau_3, z_tau_2, z_tau_1, z_region, ref_tau
    
    def step_leader(self, timestamp, v_l_target):
        # calculate the control input using the NMPC
        u_l_control, result_l = self.ato.velocity_controller(self.position, self.velocity, v_l_target)
        # take the first element for the actual time
        u_0 = u_l_control[0]
        # run the model dynamic
        s_l, v_l, a_l = self.dynamic(u_0)
        # check if a communication sending event is triggered, in positive case a message is sent
        message = self.transmitter.step(timestamp, self)
        
        return s_l, v_l, a_l, u_0, result_l, message
    
    

    # Push operation (adding elements to the top of the stack)
    def push(self, item):
        self.stack_window_channel.append(item)
        #print(f"Pushed {item} to the stack. Current stack: {self.stack_window_channel}")

    # Pop operation (removing the top element from the stack)
    def pop(self):
        if not self.is_empty():
            item = self.stack_window_channel.pop()  # pop removes the last element
            #print(f"Popped {item} from the stack. Current stack: {self.stack_window_channel}")
            return item
        else:
            print("Stack is empty, cannot pop.")
            return None
        
    # Check if the stack is empty
    def is_empty(self):
        return len(self.stack_window_channel) == 0
    
    def mle_estimator(self, delay):
        
        if len(self.stack_window_channel) >= self.len_stack:
                self.pop()
                self.push(delay)   
        else:
            self.push(delay)
            
        average_delay = sum(self.stack_window_channel)/len(self.stack_window_channel)
        
        lambda_estimated = 1/average_delay
        
        tau_bar = self.min_T - (math.log(1-self.p_channel)/lambda_estimated)
            
        return tau_bar
        
        
    
    def e_lp_computation(self, v0, u):
         
        if u >=0 :
            e_lp = (1/self.M_min)*(-self.A_max-self.B_max*v0
                               -(self.C_max*v0**2)-(self.g*self.sigma+(6*10**6)/self.rho))+(u/self.M_max)
        else:
            e_lp = (1/self.M_min)*(-self.A_max-self.B_max*v0
                               -(self.C_max*v0**2)-(self.g*self.sigma+(6*10**6)/self.rho))+(u/self.M_min)
        
        return e_lp
    
    def rlp(self, s0, v0, u_0_mpc):
        v = v0 + self.e_lp_computation(v0,u_0_mpc)*self.ts
        s = s0 + v*self.ts
        return s,v
    
    def rlp_prediction(self, u_mpc, timestamp):
        
        s_prediction = list()
        v_prediction = list()
        t_prediction = list()
        
        s = self.position
        v = self.velocity
        
        s_prediction.append(round(s,2))
        v_prediction.append(round(v,2))
        t_prediction.append(round(timestamp,2))
        
        # compute the prediction for the Robust Lower proxy 
        for u_t,i_time in zip(u_mpc,range(1,len(u_mpc))):
            s,v = self.rlp(s, v, u_t)
            s_prediction.append(round(s,2))
            v_prediction.append(round(v,2))
            t_prediction.append(round(timestamp+i_time*self.ts,2))
            
        return s_prediction, v_prediction, t_prediction
    
    def leader_rlp_predictor_block(self, message):
        
        if message is not None:
            self.rlp_s = message.s_prediction[0]
            self.rlp_v = message.v_prediction[0]
        
        s, v = self.rlp(self.rlp_s,self.rlp_v, self.em_braking)
                
        self.rlp_s = s
        self.rlp_v = v
        
    def safety_control_block(self, timestamp):
        
        # TODO import as parameters
        a_b_l = 0.7
        a_b_f = 0.65
        max_time_delay = 5
        
        # condition on time
        if self.receiver.last_message is not None:
            c_time = timestamp - self.receiver.last_message.timestamp_with_delay
        else:
            c_time = 0
            
        self.c = c_time
        
        # condition on position
        b_prime = self.d_vc - (self.rlp_s-self.position) - 0.5*((self.rlp_v**2/(2*a_b_l))-(self.velocity**2/(2*a_b_f)))
        b_second = self.d_vc - (self.rlp_s-self.position)
        b_max = max(b_prime, b_second)
        
        self.b = b_max
        
        # trigger an emergency
        if self.b >= 0 or c_time >= max_time_delay:
            self.flag_emergency = True
        else:
            self.flag_emergency = False
            
        return self.flag_emergency
            
        
                
    def get_vc_variables(self):
                
        return self.rlp_s, self.rlp_v, self.b, self.c, self.flag_emergency

    def vLeader_tau(self, vl, tau, tkBL, timestamp):
        vL_tau = None
        
        values = np.where((tkBL + timestamp[0]) < (timestamp + tau))[0]

        vL_tau = vl - abs(0.7)*tau
        
        if len(values)>0:
            idx_start = np.where(timestamp == values[0])
            idx_end = np.where(timestamp == values[-1])
            vL_tau[idx_start:idx_end]=0

        return vL_tau

    def stoppingTime(self, vl, timestamp):
        tk_b = (vl/0.7) + timestamp
        return tk_b
        
    def vFollower_tau(self, delta):
        
        return self.velocity+delta
        
    def sLeader_tau(self, sl, timestamp, vl, tkBL, tau):
        
        sL_tau = None
        
        values = np.where((tkBL + timestamp[0]) < (timestamp + tau))[0]

        sL_tau = sl+vl*tau-(1/2)*(abs(0.7)*tau**2)
        
        if len(values)>0:
            idx_start = np.where(timestamp == values[0])
            idx_end = np.where(timestamp == values[-1])
            sL_tau[idx_start:idx_end]= sl[idx_start:idx_end]+vl[idx_start:idx_end]*tau-(1/2)*(abs(0.7)*((tkBL[idx_start:idx_end] + timestamp[0])-timestamp)**2)

        return sL_tau

    def ZF_second(self, sl_tau, v_l, tau):
        
        if v_l is not list:
            v_ave = v_l
        else:
            v_ave = sum(v_l) / len(v_l)
        
        return -self.d_vc+ (sl_tau-v_ave*tau)

    def ZF_prime(self, sl_tau, vl_tau, vf_tau, v_l,tau):
        
        if v_l is not list:
            v_ave = v_l
        else:
            v_ave = sum(v_l) / len(v_l)

        return (-self.d_vc+ (sl_tau-v_ave*tau)+((vl_tau**2)/abs(2*0.7))-((vf_tau**2)/abs(2*0.7)))
        
    def ZF_tau(self, s_l, v_l, t, tau):
        
        tkBL = self.stoppingTime(v_l, t)
        
        vl_tau = self.vLeader_tau(v_l, tau, tkBL, t)
        vf_tau = self.vFollower_tau(1)

        sl_tau = self.sLeader_tau(s_l, t, v_l, tkBL, tau)
        
        
        zf_prime = self.ZF_prime(sl_tau, vl_tau, vf_tau, v_l,tau)
        zf_second = self.ZF_second(sl_tau, v_l, tau)
        # formula (24)
        return np.minimum(zf_second,zf_prime)
    
    def compute_z_FL(self, timestamp, tau):
        
        # get the last message sent from leader
        last_message_leader = self.receiver.last_message
        
        s_l = last_message_leader.s_prediction
        v_l = last_message_leader.v_prediction
        t_l = last_message_leader.t_prediction
        timestamp_l = last_message_leader.timestamp     
        
        # # Start timer
        # start_time = time.time()
        # time extracted for (28)
        if timestamp-timestamp_l <= tau:
            time_tau = [t for t in t_l if t<=timestamp]
        else:
            time_tau = [t for t in t_l if t<timestamp-tau and t <= timestamp]
            
        # formula (25)
        z_FL_tilde = [s_l[time_tau.index(t)]-self.ZF_tau(t, tau) for t in time_tau]
        
        # # End timer
        # end_time = time.time()

        # # Calculate elapsed time
        # elapsed_time = end_time - start_time
        # print("Elapsed time compute_z_FL: ", elapsed_time)    
        
        # formula (26)
        return max(z_FL_tilde)
        
    
    def compute_Z_F(self, timestamp, tau):
        
        # get the last message sent from leader
        last_message_leader = self.receiver.last_message
        
        if last_message_leader is not None:
        
            s_l = last_message_leader.s_prediction
            v_l = last_message_leader.v_prediction
            t_l = last_message_leader.t_prediction
            timestamp_l = last_message_leader.timestamp
            timestamp_delay = last_message_leader.timestamp_with_delay
            
            # # Start timer
            # start_time = time.time()
            # time extracted for (28)
            # time_ref_tau = [t for t in t_l if t<t_l[-1]-tau and t >= timestamp]
            time_ref_tau = t_l[(t_l < t_l[-1] - tau) & (t_l >= timestamp_delay)]
            
            first_element = time_ref_tau[0]
            last_element = time_ref_tau[-1]
            
            # Get the index of the element
            index_start = np.where(t_l == first_element)[0][0]
            index_end = np.where(t_l == last_element)[0][0]
            
            # Determine the time_tau array based on the condition
            
            
            s_l_1 = s_l[index_start:index_end+1]
            v_l_1 = v_l[index_start:index_end+1]
                
            # formula (25)                     
            z_FL_tilde_1 = s_l[index_start:index_end+1] - self.ZF_tau(s_l_1, v_l_1,time_ref_tau, tau) 
            
            
            # formula (27)
            #z_F = [s_l[time_ref_tau.index(t)]-self.compute_z_FL(t, tau) for t in time_ref_tau]
            z_F = s_l[index_start:index_end+1] - np.max(z_FL_tilde_1)
            # # End timer
            # end_time = time.time()

            # # Calculate elapsed time
            # elapsed_time = end_time - start_time
            # print("Elapsed time compute_Z_F: ", elapsed_time) 
            
            
            return z_F, time_ref_tau
            
        else:
            
            return None
        

