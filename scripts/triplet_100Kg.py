#!/usr/bin/env python
import rospy
from Phidget22.Phidget import *
from Phidget22.Devices.VoltageRatioInput import *
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
import time
from threading import Thread


class force_sensor():
        def __init__(self):
                self.VR = [0.0, 0.0, 0.0]
                self.Weight = [0.0, 0.0, 0.0]
                self.prev_W = [0.0, 0.0, 0.0]
                self.pub = rospy.Publisher("load_cells", Vector3, queue_size=1)
                Thread(target=self.run_sensor).start()

        def run_sensor(self):
                rate = rospy.Rate(10)
                while not rospy.is_shutdown():
                        c0 = self.Weight[0]
                        c1 = self.Weight[1]
                        c2 = self.Weight[2]
                        if(c0!=self.prev_W[0] and c1!=self.prev_W[1] and c2!=self.prev_W[2]):
                                msg = Vector3()
                                msg.x = c0
                                msg.y = c1
                                msg.z = c2
                                #print(msg.x, msg.y, msg.z)
                                (self.pub).publish(msg)
                                self.prev_W = [c0, c1, c2]
                        rate.sleep()
        
                        
def onUpdate_0(self, voltageRatio):
        global sensor
        
        w1 = 3.3373e+4
        w2 = -0.5183
        weight_ = w1*voltageRatio + w2
        force_ = weight_ * 9.81

        sensor.VR[0] = voltageRatio
        sensor.Weight[0] = weight_

        f = open("/home/dc4204/Desktop/load_cell_0.txt","a")
        f.write("\n")
        f.write(str(voltageRatio) + ", " + str(force_) + ", " + str(weight_))
        f.close()
        #print("Cell 0:,  " + str(voltageRatio)+" , "+str(weight_)) 
        

def onUpdate_1(self, voltageRatio):
        global sensor
        
        w1 = 3.3373e+4
        w2 = -0.5183
        weight_ = w1*voltageRatio + w2
        force_ = weight_ * 9.81
        
        sensor.VR[1] = voltageRatio
        sensor.Weight[1] = weight_

        f = open("/home/dc4204/Desktop/load_cell_1.txt","a")
        f.write("\n")
        f.write(str(voltageRatio) + ", " + str(force_) + ", " + str(weight_))
        f.close()
        #print("Cell 1:,  " + str(voltageRatio)+" , "+str(weight_)) 


def onUpdate_2(self, voltageRatio):
        global sensor
        
        w1 = 3.3373e+4
        w2 = -0.5183
        weight_ = w1*voltageRatio + w2
        force_ = weight_ * 9.81
        
        sensor.VR[2] = voltageRatio
        sensor.Weight[2] = weight_

        f = open("/home/dc4204/Desktop/load_cell_2.txt","a")
        f.write("\n")
        f.write(str(voltageRatio) + ", " + str(force_) + ", " + str(weight_))
        f.close()
        #print("Cell 2:,  " + str(voltageRatio)+" , "+str(weight_)) 


def prepare_log():

        f = open("/home/dc4204/Desktop/load_cell_0.txt","w+")
        f.write("VoltageRatio, Force(N), Weight(Kg)")
        f.close()
        
        f = open("/home/dc4204/Desktop/load_cell_1.txt","w+")
        f.write("VoltageRatio, Force(N), Weight(Kg)")
        f.close()

        f = open("/home/dc4204/Desktop/load_cell_2.txt","w+")
        f.write("VoltageRatio, Force(N), Weight(Kg)")
        f.close()



if __name__ == '__main__':
        rospy.init_node('hover', anonymous=True)


        sensor = force_sensor()

        prepare_log()

        #pub = rospy.Publisher("load_cell", Float64, queue_size=1)

	load_cell_0 = VoltageRatioInput()
	load_cell_1 = VoltageRatioInput()
	load_cell_2 = VoltageRatioInput()

	load_cell_0.setDeviceSerialNumber(584681)
	load_cell_0.setChannel(0)
        load_cell_0.setOnVoltageRatioChangeHandler(onUpdate_0)
	load_cell_0.openWaitForAttachment(5000)

	load_cell_1.setDeviceSerialNumber(584681)
	load_cell_1.setChannel(1)
        load_cell_1.setOnVoltageRatioChangeHandler(onUpdate_1)
	load_cell_1.openWaitForAttachment(5000)

	load_cell_2.setDeviceSerialNumber(584681)
	load_cell_2.setChannel(2)
        load_cell_2.setOnVoltageRatioChangeHandler(onUpdate_2)
	load_cell_2.openWaitForAttachment(5000)


        rospy.spin()
            
	load_cell_0.close()
        load_cell_1.close()
        load_cell_2.close()

