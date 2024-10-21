from RailwayClasses.Message import Message


class Receiver:
    def __init__(self, time_loss, duration_loss, flag_loss):
        # TODO decsrizione
               
        self.timestamp = 0
        self.last_message = None
        self.flag_loss = flag_loss
        self.duration_loss = duration_loss
        self.time_loss = time_loss
        
    def communication_loss(self, timestamp):
        
        return self.flag_loss and (timestamp >= self.time_loss and timestamp <= self.time_loss+self.duration_loss)
    
    def step(self, timestamp, message):
        self.timestamp = timestamp
        # for the first message
        if self.last_message is None:
            self.last_message = message
            
            
        
        # verify that a message is arrived
        if message is not None and not self.communication_loss(timestamp):
            #print(message.timestamp)
            if message.timestamp > self.last_message.timestamp:
                
                last_timestamp = self.last_message.timestamp
                self.last_message = message
                
                tl_KB = timestamp-last_timestamp
                
                return  timestamp-last_timestamp, tl_KB

        return None,None
    