"""
Rotator Mount Control from MIDAS Code
Skylar Wingfelder 8-23-2021

This code periodically reads and records the data from the phidgets connected to the optical box VINT Hubs.
"""

import midas
import midas.frontend
import midas.event
import collections
import math
#import ctypes
#import os
#import subprocess
from time import time
#import shlex
#import random

from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from Phidget22.Devices.Log import *
from Phidget22.LogLevel import *
from Phidget22.Devices.TemperatureSensor import *
from Phidget22.Devices.HumiditySensor import *
#not need the spatial class is recomended
#from Phidget22.Devices.Magnetometer import *
#from Phidget22.Devices.Accelerometer import *
from Phidget22.Devices.Spatial import *

ERROR_VALUE = float("inf")

#initial Phidget functions to handle errors attachment and detachment
def onAttach(self):
    SerNum=self.getDeviceSerialNumber()
    print("Humidity Phidget Attached. Serial Number: " + str(SerNum))

def onDetach(self):
    SerNum=self.getDeviceSerialNumber()
    print("Humidity Phidget Detached. Serial Number: " + str(SerNum))

def onError(self, code, description):
    print("Code: " + ErrorEventCode.getName(code))
    print("Description: " + str(description))
    print("----------")

def sign(f):
    """Python doesn't include this function cuz of a dev dissagreement"""
    if(f<0):
        return -1
    else:
        return 1

def calc_tilt(ax,ay,az):
    tilt = 90
    #calculate tilt

    #TODO: these should just be in ODB if this is still an issue when all the screws are in
    angle_offsets = [-1.01221,0.01563,0.04028]
    
    norm = math.sqrt(angle_offsets[0]*angle_offsets[0]+angle_offsets[1]*angle_offsets[1]+angle_offsets[2]*angle_offsets[2])
    angle_offsets[0]/=norm
    angle_offsets[1]/=norm
    angle_offsets[2]/=norm
    c=angle_offsets[0]
    s=-angle_offsets[2]

    ax2=c*ax-s*az
    ay2=ay
    az2=s*ax+c*az

    if (ax !=0):
        # phidget_z is defined pointing downwards
        #tilt=math.atan2(-ax2,-sign(ay2)*math.sqrt(az2*az2 + ay2*ay2))*180/math.pi+90 #if X is aligned with tilt axis!
        tilt=-math.atan2(ay,sign(ax2)*math.sqrt(az2*az2 + ax2*ax2))*180/math.pi #if X is aligned with tilt axis!
    return tilt

#create equipment class to 
class MyPeriodicEquipment(midas.frontend.EquipmentBase):

    def onSpatialData(self, ch, acceleration, angularRate, magneticField, timestamp):
        """(self) (spatial phidget object) (cartesian acceleration vector) (angular rate vector) (magnetic feild vector) (data timespamp)
        Event handler for getting the data from the data from the spatial 
        phidget as recomended by the api, if this is using too much bandwidth
        reduce the event frequency"""
        self.spaPhiData[0] = acceleration[0]
        self.spaPhiData[1] = acceleration[1]
        self.spaPhiData[2] = acceleration[2]
        
        self.spaPhiData[3] = magneticField[0]
        self.spaPhiData[4] = magneticField[1]
        self.spaPhiData[5] = magneticField[2]

        self.spaPhiData[6] = timestamp

    def __init__(self, client):
        # The name of our equipment. This name will be used on the midas status
        # page, and our info will appear in /Equipment/MyPeriodicEquipment in
        # the ODB.
        equip_name = "OpticalBox0%s" % (midas.frontend.frontend_index,)
        
        # Define the "common" settings of a frontend. These will appear in
        # /Equipment/MyPeriodicEquipment/Common. The values you set here are
        # only used the very first time this frontend/equipment runs; after 
        # that the ODB settings are used.
        default_common = midas.frontend.InitialEquipmentCommon()
        default_common.equip_type = midas.EQ_PERIODIC
        default_common.buffer_name = "SYSTEM"
        default_common.trigger_mask = 0
        default_common.event_id = 14
        default_common.period_ms = 1000
        default_common.read_when = midas.RO_ALWAYS
        default_common.log_history = 60 

        # Create the settings directory.
        humNames = ["Temperature", "Humidity"]
        spatNames = ["X Acceleration","Y Acceleration", "Z Acceleration"
            ,"X Magnetic Field","Y Magnetic Field", "Z Magnetic Field",
            "Total Magnetic Field", "Tilt","time (sec)","time (usec)"]
            
        HubSerialNumber = [354856]# set to testing value for convenience
        HumidityPhidgetHubPort = [0]
        SpatialPhidgetHubPort = [1]
        Spatial_Enable = False
        Humidity_Enable = False
        default_settings = collections.OrderedDict([ 
            ("Names", spatNames+humNames),
            ("Hub Serial Number", HubSerialNumber),
            ("Spatial Phidget Hub Port", SpatialPhidgetHubPort),
            ("Spatial Phidget Enable", Spatial_Enable),
            ("Humidity Phidget Hub Port", HumidityPhidgetHubPort),
            ("Humidity Phidget Enable", Humidity_Enable)
        ])                                                                             
        
        # You MUST call midas.frontend.EquipmentBase.__init__ in your equipment's __init__ method!
        midas.frontend.EquipmentBase.__init__(self, client, equip_name, default_common, default_settings)


        ##### hardware setup for phidget
        #Sensor List
        self.tempPhid=TemperatureSensor() #how to get other variables
        self.humPhid=HumiditySensor()
        self.spaPhid=Spatial()
        
        #Check that each Phidget is enabled
        self.Spatial_Enabled = client.odb_get("Equipment/OpticalBox0%s/Settings/Spatial Phidget Enable" % (midas.frontend.frontend_index,))
        self.Humidity_Enabled = client.odb_get("Equipment/OpticalBox0%s/Settings/Humidity Phidget Enable" % (midas.frontend.frontend_index,))

        #Get Serial Port and Hub Numbers
        HubSerialNumber[0] =  self.client.odb_get("Equipment/OpticalBox0%s/Settings/Hub Serial Number" % (midas.frontend.frontend_index,))
        SpatialPhidgetHubPort = self.client.odb_get("Equipment/OpticalBox0%s/Settings/Spatial Phidget Hub Port" % (midas.frontend.frontend_index,))
        HumidityPhidgetHubPort = self.client.odb_get("Equipment/OpticalBox0%s/Settings/Humidity Phidget Hub Port" % (midas.frontend.frontend_index,))
        
        #Create a list of the enabled phidgets
        self.phidList=[]
        
        if (self.Humidity_Enabled):
           self.phidList.append((self.tempPhid,HumidityPhidgetHubPort))
           self.phidList.append((self.humPhid,HumidityPhidgetHubPort))
        
        if (self.Spatial_Enabled):
           #ax, ay, az, mx, my ,mz
           self.spaPhiData = [0,0,0 ,0,0,0 ,0]
           self.phidList.append((self.spaPhid,SpatialPhidgetHubPort))
           self.spaPhid.setOnSpatialDataHandler(self.onSpatialData)
            
        #initialize attachment and error handlers and connect to the enabled phidgets
        for i in self.phidList:
            Phid=i[0]
            if(i[1]!=-1):
                Phid.setHubPort(i[1])
            if(HubSerialNumber[0]!=-1):
                Phid.setDeviceSerialNumber(HubSerialNumber[0])
            if(not Phid.getAttached()):
                Phid.openWaitForAttachment(1000)

            hubPort = Phid.getHubPort()
            deviceSerialNumber = Phid.getDeviceSerialNumber()
            deviceName = Phid.getDeviceName()

            attached = Phid.getAttached()

            print("Device Serial Number:",deviceSerialNumber)
            hubPortNum=i[1]
            Phid.setOnAttachHandler(onAttach)
            Phid.setOnDetachHandler(onDetach)
            Phid.setOnErrorHandler(onError)
        
        # You can set the status of the equipment (appears in the midas status page)
        self.set_status("Initialized")

    #this function runs every Common/period_ms
    def readout_func(self):
        hub_event = midas.event.Event()
        #create a bank with the humidity data if it is enabled
        humVarList=[ERROR_VALUE]*2
        if (self.Humidity_Enabled):
            tempData = self.tempPhid.getTemperature()
            humData = self.humPhid.getHumidity()
            humVarList=[tempData, humData]
            
        spatVarList=[ERROR_VALUE]*8
        #create a bank with the spatial data if it is enabled
        if (self.Spatial_Enabled):
            By=self.spaPhiData[4]
            Bz=self.spaPhiData[5]
            Bx=self.spaPhiData[3]
            Btot=math.sqrt(Bx**2+By**2+Bz**2)
            
            #AccList=self.accPhid.getAcceleration()
            Accx=self.spaPhiData[0]
            Accy=self.spaPhiData[1]
            Accz=self.spaPhiData[2]
            time = self.spaPhiData[6]
            tilt = calc_tilt(Accx,Accy,Accz)
                
            
            spatVarList=[Accx,Accy,Accz,Bx,By,Bz,Btot,tilt,time*1e-3,time*1e3]
            
        if (self.Humidity_Enabled or self.Spatial_Enabled):
            hub_event.create_bank("OB1%s" % (midas.frontend.frontend_index,), midas.TID_DOUBLE, spatVarList+humVarList)
            return hub_event
        
        return None

class MyFrontend(midas.frontend.FrontendBase):

    def __init__(self):
        # You must call __init__ from the base class.
        midas.frontend.FrontendBase.__init__(self, "feOpticalBox0%s" % (midas.frontend.frontend_index,))
        
        # You can add equipment at any time before you call `run()`, but doing
        # it in __init__() seems logical.
        self.add_equipment(MyPeriodicEquipment(self.client))

    def begin_of_run(self, run_number):
        self.set_all_equipment_status("Running", "greenLight")
        self.client.msg("Frontend has seen start of run number %d" % run_number)
        return midas.status_codes["SUCCESS"]
        
    def end_of_run(self, run_number):
        self.set_all_equipment_status("Finished", "greenLight")
        self.client.msg("Frontend has seen end of run number %d" % run_number)
       
        return midas.status_codes["SUCCESS"]

if __name__ == "__main__":

    # We must call this function to parse the "-i" flag, so it is available
    # as `midas.frontend.frontend_index` when we init the frontend object. 
    midas.frontend.parse_args()
    
    #if index is -1 break
    if (midas.frontend.frontend_index == -1):
        raise SystemExit("No Index Provided")

    # The main executable is very simple - just create the frontend object,
    # and call run() on it.
    my_fe = MyFrontend()
    my_fe.run()#import time
    
    for i in self.phidList:
        Phid=i[0]
        Phid.close() 
        

