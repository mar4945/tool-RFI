from RailwayClasses.Message import Message
import math

import numpy as np

class Transmitter:
    def __init__(self, train= None, ts=0.0, packet48 = 1.02):
        # TODO decsrizione
        
        self.train = train
        self.ts = ts
        self.packet48 = packet48
    
    # update train data 
    def update_train_data(self, train):
        
        self.train = train

    def trigger_event(self, timestamp):
        
        message = None
        # every packet48 time a message is sent, it is a trigger to trasmit data
        # if abs(timestamp % self.packet48) < self.ts or abs(timestamp % self.packet48 - self.packet48) < self.ts:
        is_triggered = math.isclose(round(timestamp % self.packet48,2), 0, abs_tol=1e-2) or math.isclose(round(timestamp % self.packet48,2), self.packet48, abs_tol=1e-2)
        if is_triggered:
            
            s_prediction, v_prediction, t_prediction = self.train.rlp_prediction(self.train.ato.u_guess, timestamp)

            
            
            message = Message(None, timestamp, np.array(t_prediction), 
                              np.array(s_prediction), np.array(v_prediction))
            
            
        return message
    
    def step(self, timestamp, train):
        
        # update data train each sampling time
        self.update_train_data(train)
        
        # check if a message is needed to sent
        message_to_transmit = self.trigger_event(timestamp)
        
        return message_to_transmit


      
            
            