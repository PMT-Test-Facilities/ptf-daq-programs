
import midas.client
import midas
import midas.frontend
import midas.event
from math import sqrt 
import numpy as np 
from math import atan2
import time 

from Phidget22.Phidget import *
from Phidget22.Devices.Accelerometer import *
from Phidget22.Devices.Spatial import *
from Phidget22.Devices.HumiditySensor import *
from Phidget22.Devices.TemperatureSensor import *
from Phidget22.Devices.Magnetometer import *

class PhidgetCon(midas.frontend.EquipmentBase):
    def __init__(self, client: midas.client.MidasClient, which=0):
        devName = "Phidget0{}".format(which)
        self.equip_name = "Phidget0{}".format(which)
    
        default_common = midas.frontend.InitialEquipmentCommon()
        default_common.equip_type = midas.EQ_PERIODIC
        default_common.buffer_name = "SYSTEM"
        default_common.trigger_mask = 0
        default_common.event_id = 13
        default_common.period_ms = 500 # not really doing anything...
        default_common.read_when = midas.RO_ALWAYS
        default_common.log_history = 60 #NOT SURE IF THIS MUST BE UNIQUE 

        self.client = client 


        midas.frontend.EquipmentBase.__init__(self, client, self.equip_name, default_common)

        self.accelerometer0 = Accelerometer()
        #self.humiditySensor0 = HumiditySensor()
        #self.temperatureSensor0 = TemperatureSensor()
        self.mag = Magnetometer()


        self.accelerometer0.setDeviceSerialNumber(673107)
        #self.humiditySensor0.setDeviceSerialNumber(673107)
        #self.temperatureSensor0.setDeviceSerialNumber(673107)
        self.mag.setDeviceSerialNumber(673107)

        self.accelerometer0.openWaitForAttachment(5000)
        #self.humiditySensor0.openWaitForAttachment(5000)
        #self.temperatureSensor0.openWaitForAttachment(5000)
        self.mag.openWaitForAttachment(5000)

    def readout_func(self):
        bfield = self.mag.getMagneticField()
        #temp = self.temperatureSensor0.getTemperature()
        acc = self.accelerometer0.getAcceleration()
        #hum = self.humiditySensor0.getHumidity()


        tilt = 90
        deno = sqrt(acc[0]**2 + acc[2]**2)
        if deno>0:
            tilt= atan2(-acc[1], deno )*180.0/3.14159
        data = [
            acc[0], 
            acc[1], 
            -acc[2], 
            bfield[0],
            bfield[1],   
            bfield[2],
            sqrt( np.sum(np.array(bfield)**2)),
            -tilt, 
            time.time(),
            time.time(),
            0, 0
        ]

        event = midas.event.Event()
        event.create_bank("PH00", midas.TID_DOUBLE, data)
        return event

class fePhidget(midas.frontend.FrontendBase):
    def __init__(self, hvmidas:PhidgetCon):
        midas.frontend.FrontendBase.__init__(self, "fePhidget")
        
        self.add_equipment(hvmidas(self.client, 0))

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
    
    #if index is -1 (not provided) break
    if (midas.frontend.frontend_index == -1):
        raise SystemExit("No Index Provided")
        
    # The main executable is very simple - just create the frontend object,
    # and call run() on it.

    my_fe = fePhidget(PhidgetCon)
    my_fe.run()
    print("closed")
    
