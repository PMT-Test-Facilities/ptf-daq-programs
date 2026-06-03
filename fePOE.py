
import midas.client
import midas
import midas.frontend
import midas.event

import numpy as np 

import py_netgear_plus

import json
import os 
_obj = open(os.path.join(
    os.path.dirname(__file__),
    "poe_data.json"
), 'rt')
this_pwd = json.load(_obj)
IP = this_pwd["IP"]
PWD = this_pwd["PWD"]

class POESwitch(midas.frontend.EquipmentBase):
    def __init__(self, client: midas.client.MidasClient):
        self.equip_name = "POESwitch"
        
        default_common = midas.frontend.InitialEquipmentCommon()
        default_common.equip_type = midas.EQ_PERIODIC
        default_common.buffer_name = "SYSTEM"
        default_common.trigger_mask = 0
        default_common.event_id = 72
        default_common.period_ms = 2500 
        default_common.read_when = midas.RO_ALWAYS
        default_common.log_history = 60 #NOT SURE IF THIS MUST BE UNIQUE 

        midas.frontend.EquipmentBase.__init__(self, client, self.equip_name, default_common)
        self.client = client

        self.sw = py_netgear_plus.NetgearSwitchConnector(IP, PWD)
        self.sw.autodetect_model()
        self.sw.get_login_cookie()
        
        self.client.msg("Connected to {}".format(self.sw.switch_model.MODEL_NAME))

        #sw.turn_off_poe_port(1) # Supported only on PoE capable models
        #sw.turn_on_poe_port(1)

    def detailed_settings_changed_func(self, path, idx, new_value):
        if "port_enable" in path:
            
            if new_value:
                self.client.odb_set("/Equipment/slowControlListener0/Readback/device_state", "Powering Up", False)
                self.sw.turn_on_poe_port(1+idx)
            else:
                mpmt_hvs = np.array(self.client.odb_get("/Equipment/slowControlListener0/Variables/pmt_hvvolval"))>50

                if any(mpmt_hvs):
                    self.client.msg("Cannot unpower mPMT!! HV still ON", True)
                    self.client.odb_set("/Equipment/POESwitch/Settings/port_enable[{}]".format(idx), True, False)
                    return 
                else:
                    self.client.odb_set("/Equipment/slowControlListener0/Readback/device_state", "Unpowered", False)
                    self.sw.turn_off_poe_port(1+idx)
            self.client.msg("Setting POE Port {} to {}".format(
                idx+1,
                "on" if new_value else "off"
            ))

    def readout_func(self):
        this_data = self.sw.get_switch_infos()
        #print(this_data.keys())

        con_speed = "port_{}_connection_speed"
        con_status = "port_{}_status"
        con_power = "port_{}_poe_power_active"
        con_power_out ="port_{}_poe_output_power"

        status = []
        speed = []
        power_active = []
        power_output =[]

        for i in range(8):
            this_port = i+1 

            status.append( this_data[con_status.format(this_port)] )
            speed.append(this_data[con_speed.format(this_port)])
            
            if this_data[con_power.format(this_port)].lower()=="on":
                power_active.append(True)
            else:
                power_active.append(False)

            power_output.append(this_data[con_power_out.format(this_port)])

        self.client.odb_set("/Equipment/POESwitch/Variables/POEE", power_active)
        self.client.odb_set("/Equipment/POESwitch/Variables/POEO", power_output)
        self.client.odb_set("/Equipment/POESwitch/Variables/POES", speed)
        self.client.odb_set("/Equipment/POESwitch/Variables/POEB", sum(power_output))

class fePOESwitch(midas.frontend.FrontendBase):
    def __init__(self, hvmidas=POESwitch):
        midas.frontend.FrontendBase.__init__(self, "fePOE")
        
        self.add_equipment(hvmidas(self.client))

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

    my_fe = fePOESwitch(POESwitch)
    my_fe.run()
    print("closed")
    