#!/usr/bin/env python
import rospy
from Phidget22.Phidget import *
from Phidget22.Devices.VoltageRatioInput import *
from std_msgs.msg import Float64
import time


def onVoltageRatioChange(self, voltageRatio):
        global pub
        
        w1 = 3.3373e+4
        w2 = -0.5183
        weight_ = w1*voltageRatio + w2
        force_ = weight_ * 9.81
        msg = Float64()
        msg.data = float(force_)
        pub.publish(msg)
        f = open("/home/dc4204/Desktop/load_cell.txt","a")
        f.write("\n")
        f.write(str(voltageRatio) + ", " + str(force_) + ", " + str(weight_))
        f.close()
        print(str(voltageRatio)+" , "+str(weight_)) 
        

if __name__ == '__main__':
        rospy.init_node('hover', anonymous=True)

        f = open("/home/dc4204/Desktop/load_cell.txt","w+")
        f.write("VoltageRatio, Force(N), Weight(Kg)")
        f.close()
        
        pub = rospy.Publisher("load_cell", Float64, queue_size=1)
        shut_down = False 

	load_cell = VoltageRatioInput()

	#Set addressing parameters to specify which channel to open (if any)
	load_cell.setDeviceSerialNumber(584675)
	load_cell.setChannel(1)
        load_cell.setOnVoltageRatioChangeHandler(onVoltageRatioChange)
	#Open your Phidgets and wait for attachment
	load_cell.openWaitForAttachment(5000)

	try:
                input("Press Enter to Stop\n")
	except (Exception, KeyboardInterrupt):
		pass

            
	load_cell.close()


