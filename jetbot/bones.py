
# traitlets are used to link events to each other, e.g. a change in a slider position with a change in motor target speed
import traitlets
# from traitlets.config.configurable import SingletonConfigurable
from traitlets import HasTraits, Float, observe

# ROS
import rospy
from geometry_msgs.msg import Twist
pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

import time
import threading

# My bot (Bones) does not expose the individual motors. Instead, it listens for Twist messages.
# define a class for Speeds, which can be instanced twice: once for forward speeds, and once for angular
# will be used when the individual speeds are to be sent

# these callbacks will be used        

# define a Robot class, which accepts some higher level motion commands, as well as instances the two speed object

class Bones(traitlets.HasTraits):   
    
    fwdspd = traitlets.Float(default_value=0.0)
    angspd = traitlets.Float(default_value=0.0)
    cmd_vel_msg = Twist()

    def __init__(self, *args, **kwargs):
        super(Bones, self).__init__(*args, **kwargs)

        self.time_last_twist = time.clock()  
        self.last_rot_spd =0.0
        self.last_fwd_spd =0.0

    def fwd_cb(self, change):
        self.cmd_vel_msg.linear.x = change.new

    def ang_cb(self,change):
        self.cmd_vel_msg.angular.z = change.new

## Twist publishes the cmd_vel_msg, which is a ROS Twist message type.
## the rate is throttled to prevent banging the arduino with too much commmunication.

    def twist(self):
        pub_cmd_vel.publish(self.cmd_vel_msg)

    def forward(self, speed):
        self.cmd_vel_msg.linear.x = speed
        self.cmd_vel_msg.angular.z = 0
        pub_cmd_vel.publish(self.cmd_vel_msg)

    def backward(self, speed):
        self.cmd_vel_msg.linear.x = -speed
        self.cmd_vel_msg.angular.z = 0
        pub_cmd_vel.publish(self.cmd_vel_msg)
            
    def left(self, speed):
        self.cmd_vel_msg.linear.x = 0
        self.cmd_vel_msg.angular.z = speed
        pub_cmd_vel.publish(self.cmd_vel_msg)
 
    def right(self, speed):
        self.cmd_vel_msg.linear.x = 0
        self.cmd_vel_msg.angular.z = -speed
        pub_cmd_vel.publish(self.cmd_vel_msg)        
        
    def stop(self):
        self.fwdspd = 0.0
        self.angspd = 0.0
        self.cmd_vel_msg.linear.x = 0
        self.cmd_vel_msg.angular.z = 0
        pub_cmd_vel.publish(self.cmd_vel_msg)
            