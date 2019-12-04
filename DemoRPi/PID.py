import time 
class PID:
    def __init__(self, Kp = 0, Ki = 0 , Kd = 0, windup = None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.integrator = 0
        self.derivative = 0

        self.last_update = time.monotonic()

        self.started = False
        
        self.use_windup = windup != None 
        if (self.use_windup):
            self.windup = windup

    def update(self, position, goal):
        error = goal - position
        
        #Proportionnal
        cmd = error * self.Kp
        if not self:
            self.started = True
            self.integrator = 0 
            self.derivative = error
            return cmd
        
        delay = time.monotonic() - self.last_update
        
        #Integral 
        self.integrator = self.integrator + error * delay
        cmd = cmd + self.integrator * self.Ki
        
        if (self.use_windup):
            self.integrator  = min(self.windup/Ki, self.integrator)
            self.integrator  = max(-self.windup/Ki, self.integrator)

        #Derivative
        cmd = cmd + (error - self.derivative)*self.Kd/delay 
        self.derivative = error

        return cmd