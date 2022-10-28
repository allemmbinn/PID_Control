import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

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
        dt = float(now - self.lastTime)

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

goal_x = 10


class turtle_PID(Node):

    def __init__(self):
        super().__init__('turtlebotControl')
        #To publish the values to the topic /cmd_vel
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        #To take values of the position and velocity from /odom
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.curr_x = 0
        self.subscription  # prevent unused variable warning
        self.linv = PID()
        self.linv.tuning(1,1,1)
        self.msg = Twist()
        self.distance = goal_x - self.curr_x
        self.msg.linear.x = self.linv.compute(self.distance)
        self.publisher_.publish(self.msg)  

    def odom_callback(self, msg):
        self.curr_x = msg.pose.pose.position.x



def main(args=None):
    rclpy.init(args=args)
    control = turtle_PID()
    rclpy.spin(control)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()