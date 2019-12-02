import time 
class PID:
    def __init__(Kp = 0, Ki = 0 , Kd = 0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.integrator = 0
        self.derivative = 0

        self.last_update = time.monotonic()

        self.started = False

    def update(position, goal):
        error = goal - position
        
        #Proportionnal
        cmd = error * Kp
        if not self:
            self.started = True
            self.integrator = 0 
            self.derivative = error
            return cmd
        
        delay = time.monotonic() - last_update
        
        #Integral 
        self.integrator = self.integrator + error * delay
        cmd = cmd + self.integrator * Ki

        #Derivative
        cmd = cmd + (error - self.derivative)/delay 
        self.derivative = error

        return cmd