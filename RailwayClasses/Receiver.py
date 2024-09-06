from RailwayClasses.Message import Message


class Receiver:
    def __init__(self):
        # TODO decsrizione
               
        self.timestamp = 0
        self.last_message = None
    
    def step(self, message):
        
        # for the first message
        if self.last_message is None:
            self.last_message = message
            
            
        
        # verify that a message is arrived
        if message is not None:
            
            if message.timestamp > self.last_message.timestamp:
                self.last_message = message
                
                return  message.timestamp_with_delay-message.timestamp

        return None
    