import pandas as pd
import rosbag
import rospy
import sys
import os
import numpy as np
import glob
import pickle
import time
import math
import argparse
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d

#from plot_all import *

def add_time(vec_time_min,vec_time_max,x):

    vec_time_min.append(np.min(x))
    vec_time_max.append(np.max(x))

    return vec_time_min,vec_time_max


def Find_time_interp(vec_time_min,vec_time_max):

    vec_time_min= np.array(vec_time_min)
    vec_time_max= np.array(vec_time_max)
    timeMin = np.max(vec_time_min)
    timeMax = np.min(vec_time_max)

    return timeMin,timeMax
def interp_2(x,y,x_new):
    # Select the interpolation method (e.g., 'linear', 'quadratic', 'cubic')
    interpolation_method = 'cubic'

    # Create an interpolation function
    f = interp1d(x, y, kind=interpolation_method)
    y_interp = f(x_new)
    return y_interp

class Vec3d:
    def __init__(self,name):
        self.x = []
        self.y = []
        self.z = []
        # self.timesec = []
        # self.timensec = []
        # self.seq = []
        # self.name = name
    def _add_(self,x,y,z):
        self.x.append(float(x))
        self.y.append(float(y))
        self.z.append(float(z))
        # self.timesec.append(float(t.secs))
        # self.timensec.append(float(t.nsecs))
        # self.seq.append(seq)
    def _interp(self,a,b):
        xvals = np.linspace(0, a, b)
        yvals =np.linspace(0, a, a)
        self.x = np.interp(xvals, yvals, self.x)
        self.y = np.interp(xvals, yvals, self.y)
        self.z = np.interp(xvals,yvals, self.z)
        self.timesec = np.interp(xvals,yvals, self.timesec)
        self.timensec = np.interp(xvals,yvals, self.timensec)

    def _panda_(self, namecsv):
        name = self.name
        nametimesec = "_timeSec"
        nametimensec = "_timeNsec"
        nameseq = "_seq"
        name1 = name+"_x"
        name2 = name+"_y"
        name3 = name+"_z"
        #name = [nametimesec,nametimensec,nameseq,name1,name2,name3]
        name = [name1,name2,name3]

        self.pd = pd.DataFrame( index=None, columns=[name])
        #self.pd[nametimesec] = self.timesec
        #self.pd[nametimensec] = self.timensec
        #self.pd[nameseq] = self.seq
        self.pd[name1] = self.x
        self.pd[name2] = self.y
        self.pd[name3] = self.z

    def cleanInterpTime(self):

        time = self.timesec - np.ones(len(self.timesec))*self.timesec[0]
        time = time + self.timensec *np.ones(len(self.timesec))*0.000000001
        SampleTime = []
        """xvals = np.linspace(0, time[-1], len(self.x))
        yvals =np.linspace(0, len(self.x), len(self.x))
        time = np.interp(xvals,yvals, time)"""
        for i in range(len(time)-1):
            SampleTime.append(time[i+1]-time[i])
        SampleTime.append(SampleTime[-1])

        return time, SampleTime


class Vec4d:
    def __init__(self,name):
        self.x = []
        self.y = []
        self.z = []
        self.w = []
        # self.seq = []
        # self.timesec = []
        # self.timensec = []
        # self.name = name

    def _add_(self,x,y,z,w):
        self.x.append(float(x))
        self.y.append(float(y))
        self.z.append(float(z))
        self.w.append(float(w))
        # self.timesec.append(float(t.secs))
        # self.timensec.append(float(t.nsecs))
        # self.seq.append(seq)
    def _interp(self,a,b):
        xvals = np.linspace(0, a, b)
        yvals = np.linspace(0, a, a)
        self.x = np.interp(xvals, yvals, self.x)
        self.y = np.interp(xvals, yvals, self.y)
        self.z = np.interp(xvals,yvals, self.z)
        self.w = np.interp(xvals,yvals, self.w)
        self.timesec = np.interp(xvals,yvals, self.timesec)
        self.timensec = np.interp(xvals,yvals, self.timensec)

    def _panda_(self,nametocsv):
        name = self.name
        nametimesec = "_timeSec"
        nametimensec = "_timeNsec"
        nameseq = "_seq"
        name1 = name+"_x"
        name2 = name+"_y"
        name3 = name+"_z"
        name4 = name+"_w"

        #name = [nametimesec,nametimensec,nameseq,name1,name2,name3,name4]
        name = [name1,name2,name3,name4]

        self.pd = pd.DataFrame( index=None, columns=[name])
        #self.pd[nametimesec] = self.timesec
        #self.pd[nametimensec] = self.timensec
        #self.pd[nameseq] = self.seq
        self.pd[name1] = self.x
        self.pd[name2] = self.y
        self.pd[name3] = self.z
        self.pd[name4] = self.w


    def cleanInterpTime(self):

        time = self.timesec - np.ones(len(self.timesec))*self.timesec[0]
        time = time + self.timensec *np.ones(len(self.timesec))*0.000000001
        SampleTime = []
        """xvals = np.linspace(0, time[-1], len(self.x))
        yvals =np.linspace(0, len(self.x), len(self.x))
        time = np.interp(xvals,yvals, time)"""
        for i in range(len(time)-1):
            SampleTime.append(time[i+1]-time[i])
        SampleTime.append(SampleTime[-1])

        return time, SampleTime


def bagToCsv(Namefile):

    bag = rosbag.Bag(Namefile+".bag")

    linearSpeedDesired  = Vec3d("linearSpeedDesired")
    linearSpeedActual  = Vec3d("linearSpeedActual")
    angularOrientationDesired = Vec4d("angularOrientationDesired")
    angularOrientationActual = Vec4d("angularOrientationDesired")

    #Variable to check if some data are empty
    IMUdata  = 0
    OPTIdata = 0
    FTdata   = 0



    List_topic = ['/passive_control/vel_quat',"/iiwa/ee_info/Vel","/iiwa/ee_info/Pose"]

    for topic, msg, t in bag.read_messages(topics=List_topic):
        if(topic == List_topic[0]):
            linearSpeedDesired._add_(msg.position.x,msg.position.y,msg.position.z)
            angularOrientationDesired._add_(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)

        elif (topic == List_topic[1]):
            linearSpeedActual._add_(msg.linear.x,msg.linear.y,msg.linear.z)
        elif (topic == List_topic[2]):
            angularOrientationActual._add_(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)

    bag.close()
    
    plt.plot(angularOrientationActual.x)
    plt.show()

    """"
    All_data_final=pd.DataFrame()


    vec_time_min = []
    vec_time_max = []

    if not len(_vrpnQuat.x) == 0:
        OPTIdata = 1

        timeOpti, sampleTime = _vrpnQuat.cleanInterpTime()
        vec_time_min,vec_time_max = add_time(vec_time_min,vec_time_max,timeOpti)

    if not len(_dataQuat.x) == 0:
        IMUdata = 1
        timeIMU, sampleTime = _dataQuat.cleanInterpTime()
        vec_time_min,vec_time_max = add_time(vec_time_min,vec_time_max,timeIMU)

    if not len(_ftforce.x) == 0:
        FTdata = 1
        timeFT, sampleTime = _ftforce.cleanInterpTime()
        vec_time_min,vec_time_max = add_time(vec_time_min,vec_time_max,timeFT)

    timeMin, timeMax= Find_time_interp(vec_time_min,vec_time_max)

    size= len(_vrpnQuat.x)
    time_interp = np.linspace(timeMin, timeMax, num=size)

    All_data_final["Time"] = time_interp-timeMin
    All_data_final["dt"]   = np.ones(len(time_interp))*All_data_final["Time"][1]

    if OPTIdata !=0:
        All_data_final["_vrpnQuat_x"] = interp_2(timeOpti,_vrpnQuat.x,time_interp)
        All_data_final["_vrpnQuat_y"] = interp_2(timeOpti,_vrpnQuat.y,time_interp)
        All_data_final["_vrpnQuat_z"] = interp_2(timeOpti,_vrpnQuat.z,time_interp)
        All_data_final["_vrpnQuat_w"] = interp_2(timeOpti,_vrpnQuat.w,time_interp)
        All_data_final["_vrpnPos_x"]  = interp_2(timeOpti,_vrpnPos.x,time_interp)
        All_data_final["_vrpnPos_y"]  = interp_2(timeOpti,_vrpnPos.y,time_interp)
        All_data_final["_vrpnPos_z"]  = interp_2(timeOpti,_vrpnPos.z,time_interp)

    if IMUdata !=0:

        All_data_final["_dataQuat_x"]   = interp_2(timeIMU,_dataQuat.x,time_interp)
        All_data_final["_dataQuat_y"]   = interp_2(timeIMU,_dataQuat.y,time_interp)
        All_data_final["_dataQuat_z"]   = interp_2(timeIMU,_dataQuat.z,time_interp)
        All_data_final["_dataQuat_w"]   = interp_2(timeIMU,_dataQuat.w,time_interp)
        All_data_final["_dataAngVel_x"] = interp_2(timeIMU,_dataAngVel.x,time_interp)
        All_data_final["_dataAngVel_y"] = interp_2(timeIMU,_dataAngVel.y,time_interp)
        All_data_final["_dataAngVel_z"] = interp_2(timeIMU,_dataAngVel.z,time_interp)

    if FTdata !=0:
        All_data_final["_ftforce_x"]  = interp_2(timeFT,_ftforce.x,time_interp)
        All_data_final["_ftforce_y"]  = interp_2(timeFT,_ftforce.y,time_interp)
        All_data_final["_ftforce_z"]  = interp_2(timeFT,_ftforce.z,time_interp)
        All_data_final["_fttorque_x"] = interp_2(timeFT,_fttorque.x,time_interp)
        All_data_final["_fttorque_y"] = interp_2(timeFT,_ftforce.y,time_interp)
        All_data_final["_fttorque_z"] = interp_2(timeFT,_ftforce.z,time_interp)

    All_data_final.to_csv(Namefile+".csv")





    print(file_name)
    csv_to_numpy(file_name, task)
    #procees()
    plot_position_ot()
    plot_quaternions()
    plot_angular_velocity()
    plot_acceleration()
    if (task):
        plot_force()
        plot_torque()
    plt.show() """
def parser():

    parser = argparse.ArgumentParser(description="bag to csv")

    parser.add_argument("-n", "--name", type=str, default='noname', nargs='?', help="The name file")
    args = parser.parse_args()
    return args

if __name__ == "__main__":
    argv = parser()

    bagToCsv(argv.name)

