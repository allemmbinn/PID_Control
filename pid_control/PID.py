from datetime import datetime

#This function exists to limit the values upto which they can change
def clamp(value, limits):
    lower,upper = limits
    if value is None:
        return None
    elif upper is not None and value > upper :
        return upper
    elif lower is not None and value < lower :
        return lower
    else:
        return value

class PID():
    def __init__(self,Kp=1.0,Ki=0.0,Kd=0.0, setPoint=0, sampleTime = 0.1, outputLimits = (None,None)):
        self.kp = Kp
        self.ki = Ki
        self.kd = Kd
        #Intial value is given to the parameter lastTime
        self.lastTime = datetime.now()
        self.sampleTime = sampleTime
        self.setPoint = setPoint
        self.outputLimits = outputLimits
        
        self.proportional = 0 
        self.integral = 0
        self.derivative = 0 

    def tuning(self, Kp, Ki, Kd):
        self.kp = Kp
        self.ki = Ki
        self.kd = Kd

    def compute(self,input):
        now = datetime.now()
        dt = now - self.lastTime

        if(dt>=self.sampleTime):     
            #Get Setpoint and Input
            error = self.setPoint - input
            
            self.proportional = self.kp * error
            self.integral +=  self.ki * error * dt
            self.derivative = -self.kd * (input - self.lastInput) / dt
            
            self.proportional = clamp(self.proportional, self.outputLimits)
            self.integral = clamp(self.integral, self.outputLimits)
            self.derivative = clamp(self.derivative, self.outputLimits)

            output =  self.proportional + self.integral + self.derivative
            output = clamp(output,self.outputLimits)

            self.lastTime = now
            self.lastInput = input
            self.lastOutput = output
            return output

    #To update the setPoint at every Position
    def update(self, setPoint):
        self.setPoint = setPoint