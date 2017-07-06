#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from threading import Lock
from trex_driver import trex_driver as td


class PololuTrexNode:
    pub_cur = [None, None]
    
    def __init__(self, device, config, index=0):
        
        self.cfg = config.cfg
        self.device = device
        # Publisher and subscriber
        self.pub_cur[0] = rospy.Publisher("~current%d" % (index+0), Float64, queue_size=10)
        self.pub_cur[1] = rospy.Publisher("~current%d" % (index+1), Float64, queue_size=10)
        self.sub_cmd0 = rospy.Subscriber("~command%d" % (index+0), Float64, self.cb_cmd, index+0)
        self.sub_cmd1 = rospy.Subscriber("~command%d" % (index+1), Float64, self.cb_cmd, index+1)

        # Trex Device
        self.trex = td.PololuTrex(device,config)
        self.mutex = Lock()
        
        # Timer
        rospy.Timer(rospy.Duration(1.0 / self.cfg['rate']), self.cb_timer)
    
         
    def cb_timer(self, event):
        
        if not self.trex.is_open: return       
        
        self.mutex.acquire()
        try:    
            currents = self.trex.getCurrents()
            self.pub_cur[0].publish(Float64(currents[0]))
            self.pub_cur[1].publish(Float64(currents[1]))
        except Exception as e:
                rospy.logerr("Could not read motor currents: %s" % (e))
        finally:
            self.mutex.release()

        
    def cb_cmd(self, msg, motor):
        self.mutex.acquire()
        try: 
            self.trex.setPWM(motor%2, msg.data)
            rospy.loginfo("Received PWM command on command%d (%s): %f" % (motor, self.device, msg.data))
        except Exception as e:
                rospy.logerr("Could not send motor command: %s" % (e))
        finally:
            self.mutex.release()

        
    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.pwmValue[0] is not None:
                self.trex.setPWM(0,self.pwmValue[0])
                print "SET PWM 1 %f" % self.pwmValue[0]
            if self.pwmValue[1] is not None:
                self.trex.setPWM(1,self.pwmValue[1])
            rate.sleep()
        

def shutdown_hook():
  print "shutdown time!"


if __name__ == '__main__':
	rospy.init_node('trex_driver', anonymous=False)
	roscfg = td.RosConfigPololuTrex()
	
     	i=0
	for dev in roscfg.cfg["devices"]:
		PololuTrexNode(dev, roscfg,i)
		i=i+2
	rospy.on_shutdown(shutdown_hook)
	rospy.spin()
