import os
from slack_bolt import App
from slack_bolt.adapter.socket_mode import SocketModeHandler

import time

import midas.client
import midas
import midas.frontend
import midas.event
import collections 

from get_status import get_status

import json 

_obj = open("tokens.json")
data = json.load(_obj)
_obj.close()

bot_token = data["bot_token"]
app_token = data["app_token"]
channelid = data["channelid"]


app = App(token=bot_token)
test = SocketModeHandler(app, app_token)

class SlackBot(midas.frontend.EquipmentBase):
    def __init__(self, client : midas.client.MidasClient, sockethandle):
        devName = "slackbot"
        equip_name = "slackbot"


        default_common = midas.frontend.InitialEquipmentCommon()
        default_common.equip_type = midas.EQ_PERIODIC
        default_common.buffer_name = "SYSTEM"
        default_common.trigger_mask = 0
        default_common.event_id = 16
        default_common.period_ms = 5000
        default_common.read_when = midas.RO_ALWAYS
        default_common.log_history = 60 #NOT SURE IF THIS MUST BE UNIQUE 


        default_settings = collections.OrderedDict([  
            ("dev",devName),

        ]) 
        self.client = client 
        self.app = sockethandle

        midas.frontend.EquipmentBase.__init__(self, client, equip_name, default_common, default_settings)

        self.laser_status_old, self.wiener_status_old = get_status()
        self._failure = False


    def send_message(self, message):
        self.app.client.chat_postMessage(channel=channelid, text=message)

    def readout_func(self):
        
        try:
            laser_status, wiener_status = get_status()
        except Exception:
            if not self._failure:
                self.send_message("MIDAS has crashed!")
            self._failure = True 
            return  


        if laser_status != self.laser_status_old and laser_status != "Pend On":
            self.send_message(f"The laser has been turned {laser_status}")
            self.laser_status_old = laser_status
        if wiener_status != self.wiener_status_old and wiener_status != "Pend On":
            self.send_message(f"The HV has been turned {wiener_status}")
            self.wiener_status_old = wiener_status



class MyFrontend(midas.frontend.FrontendBase):
    """
    """
    def __init__(self,myslackbot:SlackBot, socket):
        # You must call __init__ from the base class.
        midas.frontend.FrontendBase.__init__(self, "feSlackBot")
        self.slackbot = myslackbot(self.client, socket)
        self.add_equipment(self.slackbot)

        # end of move handle
        
    def begin_of_run(self, run_number):
        """
            Any setup for the beginning of the run 
        """
        self.slackbot.send_message("Starting Run {}".format(run_number)) 
    def end_of_run(self, run_number):
        """
            Any cleanup....
        """
        self.slackbot.send_message("Ending Run {}".format(run_number)) 



if __name__ == "__main__":

    # We must call this function to parse the "-i" flag, so it is available
    # as `midas.frontend.frontend_index` when we init the frontend object. 
    midas.frontend.parse_args()
    
    #if index is -1 (not provided) break
    if (midas.frontend.frontend_index == -1):
        raise SystemExit("No Index Provided")
        
    # The main executable is very simple - just create the frontend object,
    # and call run() on it.

    my_fe = MyFrontend(SlackBot, app)
    my_fe.run()
    print("closed")
    