"""
Rotator Mount Control from MIDAS Code
Skylar Wingfelder 8-23-2021
This code reads and reports the position of the rotator mount periodically and watches the ODB to see if the 
Destination changes. If so it moves the mount to the new position.
"""

import midas
import midas.frontend
import midas.event

import collections
import time
import serial

#Code which converts a float corresponding to an angle to an 8 character hexidecimal number which indicates how many counts correspond to this angle in the rotation mount software
def deg2tx (angle):
    
    print("d2t angle: ",angle) #print commands for trouble shooting all can be removed
    degPerCount = 0.0025 # degrees per count from spec sheet
    counts=int(angle/degPerCount) # integer of the number in counts 
    print("d2t counts: ", counts)
    
    
    
    #if the angle is negative the 2s complement must be found before converting the number to hexidecimal
    if (counts < 0):        
        counts=counts*-1 #remove negative sign
        F8=int('FFFFFFFF',16) #Maximum hexidecimal number 
        twoscomp=F8-counts #subtract counts from max number
        twoscomp=twoscomp+1 # add 1
        hexstr=hex(twoscomp)[2:] #convert to hex but remove the first two 0x characters of the string
        
    #if positive or zero just convert to hexidecimal    
    else: 
        hexstr=hex(counts)[2:] #convert to hex but remove the first two 0x characters of the string
        for i in range(8-len(hexstr)):
            hexstr="0"+hexstr #add zeros to the front of the string until it is 8 characters long
       
    print("d2t hxstr: ",hexstr)
    
    return hexstr

#performs the reverse operation of deg2tx
def rx2deg (reply):

    print("r2d reply: ",reply)
    degPerCount = 0.0025
    F8=int('FFFFFFFF',16)
    
    reply=str(reply)[5:13]#convert reply string from bytes literal and take only the char that correspond to position
    print("r2d abbrvreply: ",reply)
    print(reply)
    angle=999.9 #if there is an error with the reply the angle will be 999. This can be improved by raising an error

    #if positiove or 0 convert to int and then calculate the number of degrees that correspond to the count
    if (reply[0]=="0"):
        counts=int(reply,16)
        angle=counts*degPerCount
        
    #if negative reverse the 2s complement syntax then convert to int and degrees
    if (reply[0]=="F"):
        repint=int(reply,16)
        twosuncomp=repint-1
        counts=(F8-twosuncomp)*-1
        angle=counts*degPerCount

    angle=round(angle,3)
    print("r2d: ",angle)
    return angle
    

def serial_move_to(dist):
        #TODO: we didn't test this yet since we needed power for the usb hub
        countstr=deg2tx(dist)
        mvcmd="0ma"+countstr.upper()
        ser.write(mvcmd.encode())
        print("command sent: ",mvcmd)
        time.sleep(1)#wait for reply
        posreply=ser.readline() #possibly use readline
        return rx2deg(posreply)

#Define a MIDAS equipment class for the rotation mount
class MyPeriodicEquipment(midas.frontend.EquipmentBase):    
     
    def __init__(self, client):
        # The name of our equipment. This name will be used on the midas status
        # page, and our info will appear in /Equipment/MyPeriodicEquipment in
        # the ODB
        self.equip_name = "RotatorMount0%s" % (midas.frontend.frontend_index,)
        
        # Define the "common" settings of a frontend. These will appear in
        # /Equipment/MyPeriodicEquipment/Common. The values you set here are
        # only used the very first time this frontend/equipment runs; after 
        # that the ODB settings are used.
        default_common = midas.frontend.InitialEquipmentCommon()
        default_common.equip_type = midas.EQ_PERIODIC
        default_common.buffer_name = "SYSTEM"
        default_common.trigger_mask = 0
        default_common.event_id = 16
        default_common.period_ms = 6000
        default_common.read_when = midas.RO_ALWAYS
        default_common.log_history = 60 


        # Create the settings directory.  
        devName = "RotatorMount1" # this was a dummy variable and can be deleted 
        Destination = 0.01 #set the angle that the Mount should travel to

        default_settings = collections.OrderedDict([  
            ("dev",devName),
            ("Destination", Destination),

        ]) 
        
        
        default_variables = collections.OrderedDict([  
            ("Position",[0.0]),
        ]) 
         
        
        # You MUST call midas.frontend.EquipmentBase.__init__ in your equipment's __init__ method!
        #EquipmentBase doesn't call for default_variables in the documentation
        midas.frontend.EquipmentBase.__init__(self, client, self.equip_name, default_common, default_settings)#, default_variables)

        
        #hardware setup for rotator mount
        global ser #define serial port globally so all functions can access the same one
        ser = serial.Serial(
            port='/dev/ttyUSB0',
            baudrate=9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS)

        print("post-open serial")
        
	
        """
        #initialize mount at address 1 and get some data
        initcmd="0in\r\n"
        self.ser.write(initcmd.encode())
        print("command sent")
        time.sleep(1)
        init_rep=self.ser.readline() #possibly use readline
        print(init_rep)
        
        self.ser.close()
        
        print("init")
        """

        #go home - device must go home before staring or else it cannont report the correct positions
        homecmd="0ho0\r\n" #communication command so motor at address 0 goes home cw
        ser.write(homecmd.encode())#encode the string and then send it to the serial port
        print("command sent")
        time.sleep(1)# I found that a 1 second sleep was needed to wait for the reply this is longer thanthe baudrate would suggest
        posreply=ser.readline() #read most recent line in serial port file
        pos=rx2deg(posreply) #convert to a float value
        dest=self.client.odb_get("/Equipment/RotatorMount0%s/Settings/Destination" % (midas.frontend.frontend_index,))
        pos=serial_move_to(dest)
        self.client.odb_set("/Equipment/RotatorMount0%s/Variables/Position[0]" % (midas.frontend.frontend_index,),pos) # set the odb variable

        self.set_status("Initialized")
       
       
    #Reports devices current position  
    def readout_func(self):
 
        event = midas.event.Event()
        #var=[self.client.odb_get("/Equipment/RotatorMount00/Variables/Position[0]")]
        getposcmd="0gp\r\n" #get device position
        ser.write(getposcmd.encode())
        print("command sent")
        time.sleep(1)
        posreply=ser.readline() #read most recent line in serial port file
        var=rx2deg(posreply) #convert to a float value
        self.client.odb_set("/Equipment/RotatorMount0%s/Variables/Position[0]" % (midas.frontend.frontend_index,),var) # set the odb variable
        
        event.create_bank("RM0%s" % (midas.frontend.frontend_index,), midas.TID_FLOAT, [var] )

        print("read")
        return event


#create a class for the frontend on the MIDAS page which interfaces with the equipment

class MyFrontend(midas.frontend.FrontendBase):

    def __init__(self,MyPeriodicEquipment):
        # You must call __init__ from the base class.
        midas.frontend.FrontendBase.__init__(self, "feRotatorMount0%s" % (midas.frontend.frontend_index,))
        
        # You can add equipment at any time before you call `run()`, but doing
        # it in __init__() seems logical.
        self.add_equipment(MyPeriodicEquipment(self.client))
        #print(MyPeriodicEquipment.a)
        #self.ser=MyPeriodicEquipment.ser
		
        #Setting up hotlink/watching the odb destination variable to see if it changes
        #when the destination changes in the odb the odb_callback function executes
        self.client.odb_watch("/Equipment/RotatorMount0%s/Settings/Destination" % (midas.frontend.frontend_index,),self.my_odb_callback)

    def begin_of_run(self, run_number):
        self.set_all_equipment_status("Running", "greenLight")
        self.client.msg("Frontend has seen start of run number %d" % run_number)
        return midas.status_codes["SUCCESS"]
        
    def end_of_run(self, run_number):
        self.set_all_equipment_status("Finished", "greenLight")
        self.client.msg("Frontend has seen end of run number %d" % run_number)
        self.odb_stop_watching("/Equipment/RotatorMount0%s/Settings/Destination" % (midas.frontend.frontend_index,))
        return midas.status_codes["SUCCESS"]
        

    #If destination changes move the rotator mount to that position and update the new position in the ODB
    def my_odb_callback(self,client, path, odb_value):
        time.sleep(1) #not disrupt other communications if they are happening
        dest=self.client.odb_get("/Equipment/RotatorMount0%s/Settings/Destination" % (midas.frontend.frontend_index,))
        pos=serial_move_to(dest)
        self.client.odb_set("/Equipment/RotatorMount0%s/Variables/Position[0]" % (midas.frontend.frontend_index,),pos)

        
        
if __name__ == "__main__":

    # We must call this function to parse the "-i" flag, so it is available
    # as `midas.frontend.frontend_index` when we init the frontend object. 
    midas.frontend.parse_args()
    
    #if index is -1 (not provided) break
    if (midas.frontend.frontend_index == -1):
        raise SystemExit("No Index Provided")
        
    # The main executable is very simple - just create the frontend object,
    # and call run() on it.
    my_fe = MyFrontend(MyPeriodicEquipment)
    my_fe.run()
    ser.close()