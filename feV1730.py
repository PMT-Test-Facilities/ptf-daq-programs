import caen_v1730 
import numpy as np 

import midas
import midas.frontend
import midas.event

import collections
import time 

import caen_v1730 

# 100
class v1730Optic(midas.frontend.EquipmentBase):
    def __init__(self, client):
        devName = "v1730"
        equip_name = "v1730"

        default_common = midas.frontend.InitialEquipmentCommon()
        default_common.equip_type = midas.EQ_POLLED
        default_common.buffer_name = "SYSTEM"
        default_common.trigger_mask = 0
        default_common.event_id = 100
        default_common.period_ms = 1000
        default_common.read_when = midas.RO_RUNNING
        default_common.log_history = 60 

        midas.frontend.EquipmentBase.__init__(self, client, equip_name, default_common)


        self.start_key = "EOM"
        self.end_key = "BONM"
        self.is_reading = False 

        self.digi = caen_v1730.V1730Digitizer(
            link_type=1,
            link_num = 0,
            conet_node = 0,
            base_addr = 0
        )

        self.digi.configure_external_trigger()

    def runstart(self):
        self.digi.start_acquisition()

    def runend(self):
        self.digi.stop_acquisition() 

    def readout_func(self):
        event = midas.event.Event()

        pmt20, monitor = self.digi.pull_charge(1000)

        event.create_bank("QMON", midas.TID_INT, np.array(monitor).astype(int))
        event.create_bank("QPMT", midas.TID_INT, np.array(pmt20).astype(int))

        return event 
    
    def poll_func(self):
        event = self.client.receive_event(self._buffer_handle, async_flag=True)
        if event is not None:
            for bank in event.banks.values():
                if bank[:3]==self.start_key:
                    self.is_reading = True 
                elif bank==self.end_key:
                    self.is_reading = False 

        return self.is_reading

class feV1730(midas.frontend.FrontendBase):
    """
    """
    def __init__(self):
        # You must call __init__ from the base class.
        midas.frontend.FrontendBase.__init__(self, "feV1730_%s" % (midas.frontend.frontend_index,))
        
        self.v1730=v1730Optic(self.client)
        self.add_equipment(self.v1730)


    def begin_of_run(self, run_number):
        self.set_all_equipment_status("Running", "greenLight")
        self.v1730.runstart()
        self.client.msg("Frontend has seen start of run number %d" % run_number)
        return midas.status_codes["SUCCESS"]
        
    def end_of_run(self, run_number):
        self.set_all_equipment_status("Finished", "greenLight")
        self.v1730.runend()
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
    my_fe = feV1730()
    my_fe.run()
    print("closed")
    
