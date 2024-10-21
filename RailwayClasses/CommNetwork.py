import random
import math
import matplotlib.pyplot as plt
import numpy as np


class CommNetwork:
    def __init__(self, exp_lambda, min_delay, time_loss, duration_loss, flag_loss):
        
        self.exp_lamda = exp_lambda
        self.min_delay = min_delay
        self.stored_message = list()
        
        self.time_loss = time_loss
        self.duration_loss = duration_loss
        self.flag_loss = flag_loss
        
        
    def add_delay(self):
        
        # Generate exponential distribution sample
        generated_number = np.random.exponential(1/self.exp_lamda, 1) + self.min_delay
        
        return np.round(generated_number.item(),2)
    
    def set_param_channel(self, param):
        
        self.exp_lamda = param
        
    def communication_loss(self, timestamp):
        
        return self.flag_loss and (timestamp >= self.time_loss and timestamp <= self.time_loss+self.duration_loss)
    
    def step(self, timestamp, leader_message):
        
        message_to_send = list()
        
        # receive the leader message and add the delay
        if leader_message is not None:
            leader_message.timestamp_with_delay = leader_message.timestamp + self.add_delay()
            
            # Here the function to reproduce the loss communication, the message arrive but never stored
            if not self.communication_loss(timestamp):
                self.stored_message.append(leader_message)
        
        # loop for each message, if the time stamp is greater than the timestamp plus delay send the message
        for message in self.stored_message:
            
            if message.timestamp_with_delay <= timestamp:
                
                message_to_send.append(message)
                self.stored_message.remove(message)

        # send the message which have the most updated timestamp
        return max(message_to_send, key=lambda obj: obj.timestamp) if len(message_to_send)>0 else None 
    
    
