
import numpy as np
import opengen as og
import sys
import os

# Ottiene il percorso del file corrente
path_controller = os.path.dirname(os.path.abspath(__file__))
print(path_controller)

# These controllers are for the Cruise virtual coupling controller
sys.path.insert(1, path_controller+"/Controller/controller_tau_1_new")
from Train import Train
import controller_tau_1_new # type: ignore

sys.path.insert(1, path_controller+"/Controller/controller_tau_2_new")
import controller_tau_2_new # type: ignore

sys.path.insert(1, path_controller+"/Controller/controller_tau_3_new")
import controller_tau_3_new # type: ignore

# This controller is for the Leader, basically it try to follow a desidered velocity
sys.path.insert(1, path_controller+"/Controller/controller_solver")
import controller_solver # type: ignore


class ATO:
    
    # constants
    U_FACTOR_VC = 10000
    U_FACTOR_VELOCITY = 1000
    
    def __init__(self, N = None, ts=None, s_1 = None, s_2=None, s_3=None):
             
        self.N = N
        self.ts = ts
        self.nu = 1
        self.u_guess = [0]*N
        self.u_past = 5
        
        self.z_tau_1 = np.full(self.N*20, s_1)
        self.time_ref_1 = np.round(np.arange(0, self.N*20*self.ts, self.ts), 2) 
        self.z_tau_2 = np.full(self.N*20, s_2)
        self.time_ref_2 = np.round(np.arange(0, self.N*20*self.ts, self.ts), 2) 
        self.z_tau_3 = np.full(self.N*20, s_3)
        self.time_ref_3 = np.round(np.arange(0, self.N*20*self.ts, self.ts), 2) 
        
        self.emergency_braking = 0
        
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
        
        h = (i+N)-i
        
        
        
        # used to check in whic region z we are
        z_region = None
        # switched controllers based on z_taus references
        if s_f < self.z_tau_3[idx_ref]:
            
            ref = self.z_tau_2[i:(i+N)]
        
            # create the parametrized vector for NMPC
            P = [s_f]+[v_f] + [v_l] + [self.u_past] + ref.tolist()
            ref_tau = ref[0]
            
            [uMPC,result] = self.k_tau_3(P)
            z_region = 1
        elif s_f > self.z_tau_1[idx_ref]:
            ref = self.z_tau_2[i:(i+N)]
        
            # create the parametrized vector for NMPC
            P = [s_f]+[v_f] + [v_l] + [self.u_past] + ref.tolist()
            ref_tau = ref[0]
            z_region = 3
            [uMPC,result] = self.k_tau_1(P)
        else:
            ref = self.z_tau_1[i:(i+N)]
        
            # create the parametrized vector for NMPC
            P = [s_f]+[v_f] + [v_l] + [self.u_past] + ref.tolist()
            ref_tau = ref[0]
            z_region = 2
            [uMPC,result] = self.k_tau_2(P)
        
        # collect the past solution used as guess for the next optimization problem and store the last input computed
        
        
        self.u_past = uMPC[0]
        self.u_guess = [u*self.U_FACTOR_VC for u in uMPC]
        
        return self.u_guess, result, self.z_tau_3[idx_ref], self.z_tau_2[idx_ref], self.z_tau_1[idx_ref], z_region, ref_tau
    
    def set_emergency_braking(self, brake):
        self.emergency_braking = brake
    
    def emergency_controller(self):
        # TODO modificare costante
        u_e = self.emergency_braking - 30000
        #u_e = -370000
        
        if u_e < -370000:
            u_e = -370000
        
        return u_e
    
    def set_u_past(self, u):
        self.u_past = u
    
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
        

