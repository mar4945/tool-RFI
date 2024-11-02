
import numpy as np
import opengen as og
import sys
import os

# Ottiene il percorso del file corrente
path_controller = os.path.dirname(os.path.abspath(__file__))
parent_directory = os.path.dirname(path_controller)


# These controllers are for the Cruise virtual coupling controller
sys.path.insert(1, parent_directory+"/Controller/controller_tau_1_new")
from RailwayClasses.Train import Train
import controller_tau_1_new # type: ignore

sys.path.insert(1, parent_directory+"/Controller/controller_tau_2_new")
import controller_tau_2_new # type: ignore

sys.path.insert(1, parent_directory+"/Controller/controller_tau_3_new")
import controller_tau_3_new # type: ignore

# This controller is for the Leader, basically it try to follow a desidered velocity
sys.path.insert(1, parent_directory+"/Controller/controller_solver")
import controller_solver # type: ignore

sys.path.insert(1, parent_directory+"/Controller/controller_tau_1bis_new")
import controller_tau_1bis_new # type: ignore


class ATO:
    
    # constants
    U_FACTOR_VC = 10000
    U_FACTOR_VELOCITY = 1000
    
    def __init__(self, N = None, ts=None, s_1 = None, s_2=None, s_3=None, os1=None):
             
        self.N = N
        self.ts = ts
        self.nu = 1
        self.u_guess = [0]*N
        self.u_past = 0
        self.os1 = os1
        
        self.z_tau_1 = np.full(self.N*20, s_1)
        self.time_ref_1 = np.round(np.arange(0, self.N*20*self.ts, self.ts), 2) 
        self.z_tau_2 = np.full(self.N*20, s_2)
        self.time_ref_2 = np.round(np.arange(0, self.N*20*self.ts, self.ts), 2) 
        self.z_tau_3 = np.full(self.N*20, s_3)
        self.time_ref_3 = np.round(np.arange(0, self.N*20*self.ts, self.ts), 2) 
        
        self.emergency_braking = 0
        self.past_ref = -1
        
    def set_z_tau_ref(self,z1,t1,z2,t2,z3,t3):
        
        self.z_tau_1 = z1
        self.time_ref_1 = t1
        self.z_tau_2 = z2
        self.time_ref_2 = t2
        self.z_tau_3 = z3
        self.time_ref_3 = t3
        

    # Transition controller from L3 -> VC, it is ised to reach the ref z_tau2
    def k_tau_3(self, P ):
        
        solver = controller_tau_1_new.solver()
        result = solver.run(p=P,
                            initial_guess=self.u_guess)
        
        u_star = result.solution
        return u_star[0:self.nu*self.N:self.nu], result
    
    # Transition controller from L3 -> VC, it is ised to reach the ref z_tau2
    def k_tau_3bis(self, P ):
        
        solver = controller_tau_1bis_new.solver()
        result = solver.run(p=P,
                            initial_guess=self.u_guess)
        
        if result is None:
            return self.k_tau_3(P)
        
        u_star = result.solution
        return u_star[0:self.nu*self.N:self.nu], result
    
    # cruise controller which allow to follow the ref z_tau2
    def k_tau_2(self, P):
        
        solver = controller_tau_2_new.solver()
        result = solver.run(p=P, initial_guess=self.u_guess)

        u_star = result.solution
        return u_star[0:self.nu*self.N:self.nu], result
    
    # prudent controller which brakes to catch up the ref z_tau2
    def k_tau_1(self, P):
        
        solver = controller_tau_3_new.solver()
        result = solver.run(p=P, initial_guess=self.u_guess)
        
        u_star = result.solution
        return u_star[0:self.nu*self.N:self.nu], result

    # controller used to follow the desired velocity
    def k_leader(self, P):
        
        solver = controller_solver.solver()
        result = solver.run(p=P, initial_guess=self.u_guess)
        
        u_star = result.solution
        return u_star[0:self.nu*self.N:self.nu], result
        
    # Controller for the virtual coupling
    def cruise_virtual_coupling(self, s_f, v_f, v_l, timestamp, message):
        
        idx_ref = np.where(self.time_ref_1 == timestamp)[0][0]
        
        N = self.N
        
        i = int(idx_ref)
                   
        
        # used to check in whic region z we are
        z_region = None
        
        # case when the message doesn't arrive so the stack will overflow
        if idx_ref >= len(self.z_tau_3):
            z_tau_3_ref =  self.z_tau_3[-1]
        else:
            z_tau_3_ref =  self.z_tau_3[idx_ref]
            
        if idx_ref >= len(self.z_tau_2):
            z_tau_2_ref =  self.z_tau_2[-1]
        else:
            z_tau_2_ref =  self.z_tau_2[idx_ref]
            
        if idx_ref >= len(self.z_tau_1):
            z_tau_1_ref =  self.z_tau_1[-1]
        else:
            z_tau_1_ref =  self.z_tau_1[idx_ref]
        # switched controllers based on z_taus references
        if s_f <= z_tau_3_ref:
            
            ref = self.z_tau_2[i:(i+N)]
            ref = self.add_ref(ref.tolist())
        
            # create the parametrized vector for NMPC
            P = [s_f]+[v_f] + [v_l] + [self.u_past] + ref
            ref_tau = ref[0]
            
            # controller added to improve the performance of the control system
            if  timestamp< 1500 and self.os1:
                [uMPC,result] = self.k_tau_3(P)
                z_region = 3
            else:
                [uMPC,result] = self.k_tau_3bis(P)
                z_region = 3
            
           
        elif s_f > z_tau_1_ref:
            ref = self.z_tau_2[i:(i+N)]
            ref = self.add_ref(ref.tolist())
        
            # create the parametrized vector for NMPC
            P = [s_f]+[v_f] + [v_l] + [self.u_past] + ref
            
            ref_tau = ref[0]
            z_region = 1
            [uMPC,result] = self.k_tau_1(P)
        else:
            ref = self.z_tau_1[i:(i+N)]
            ref = self.add_ref(ref.tolist())
            
            # create the parametrized vector for NMPC
            P = [s_f]+[v_f] + [v_l] + [self.u_past] + ref
            ref_tau = ref[0]
            z_region = 2
            [uMPC,result] = self.k_tau_2(P)
        
        # collect the past solution used as guess for the next optimization problem and store the last input computed
        
        # initialize the controller K2, this is an initial condition in this scenario
        if timestamp<3:
            [uMPC,result] = self.k_tau_2(P)
            z_region = 2
        
        self.u_past = uMPC[0]
        self.u_guess = [u*self.U_FACTOR_VC for u in uMPC]
        
        self.past_ref = ref_tau
        
        return self.u_guess, result, z_tau_3_ref, z_tau_2_ref, z_tau_1_ref, z_region, ref_tau
    
    def set_emergency_braking(self, brake):
        self.emergency_braking = brake
    
    def emergency_controller(self):
        # TODO modificare costante
        u_e = self.emergency_braking - 10000
        #u_e = -370000
        
        if u_e < -370000:
            u_e = -370000
        
        return u_e
    
    def set_u_past(self, u):
        self.u_past = u
        
    # this function allocate element to the ref to not have problem on the controller
    def add_ref(self, lista):

        if len(lista) == 0:
            lista = [self.past_ref] * self.N
        elif len(lista) < self.N:
            ultimo_elemento = lista[-1]
            elementi_da_aggiungere = [ultimo_elemento] * (self.N - len(lista))
            lista.extend(elementi_da_aggiungere)
        return lista
    
    # NMPC controller used to follow the desired velocitu
    def velocity_controller(self, s_f, v_f, v_l_target):
        
        # cretae velocity reference for the leader
        ref = [v_l_target]*self.N
        
        # create the parametrized vector for NMPC
        P = [s_f]+[v_f] + [self.u_past] + ref
        
        [uMPC,result] = self.k_leader(P)

        # collect the past solution used as guess for the next optimization problem and store the last input computed
        self.u_past = uMPC[0]
        self.u_guess = [u*self.U_FACTOR_VELOCITY for u in uMPC]
        
        return self.u_guess, result
        

