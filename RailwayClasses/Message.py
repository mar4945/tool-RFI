class Message:
    def __init__(self, timestamp_with_delay=None, timestamp=None, t_prediction=None, s_prediction=None, v_prediction=None,  ):
        
        self.timestamp = timestamp
        self.timestamp_with_delay = timestamp_with_delay
        self.t_prediction = t_prediction
        self.s_prediction = s_prediction
        self.v_prediction = v_prediction
        
    def get_packet(self):
        
        return self.timestamp, self.timestamp_list, self.s_prediction, self.v_prediction
    
    def __str__(self):
        return (f"timestamp: {self.timestamp} s, "
                f"timestamp_with_delay: {self.timestamp_with_delay} s, "
                f"t_prediction: {self.t_prediction} s"
                f"s_prediction: {self.s_prediction} m, "
                f"v_prediction: {self.v_prediction} m/s"
                )

