#!/usr/bin/env python

"""Copyright 2010 Phidgets Inc.
This work is licensed under the Creative Commons Attribution 2.5 Canada License. 
To view a copy of this license, visit http://creativecommons.org/licenses/by/2.5/ca/
"""

__author__ = 'Adam Stelmack'
__version__ = '2.1.8'
__date__ = 'May 17 2010'

#Basic imports
from ctypes import *
import sys
import math
#from ROOT import TGraph, TCanvas
#Phidget specific imports
from Phidgets.Phidget import Phidget
from Phidgets.PhidgetException import PhidgetErrorCodes, PhidgetException
from Phidgets.Events.Events import SpatialDataEventArgs, AttachEventArgs, DetachEventArgs, ErrorEventArgs
from Phidgets.Devices.Spatial import Spatial, SpatialEventData, TimeSpan

#Create an accelerometer object
try:
    spatial = Spatial()
except RuntimeError as e:
    print("Runtime Exception: %s" % e.details)
    print("Exiting....")
    sys.exit(1)

#Information Display Function
def DisplayDeviceInfo():
    print("|------------|----------------------------------|--------------|------------|")
    print("|- Attached -|-              Type              -|- Serial No. -|-  Version -|")
    print("|------------|----------------------------------|--------------|------------|")
    print("|- %8s -|- %30s -|- %10d -|- %8d -|" % (spatial.isAttached(), spatial.getDeviceName(), spatial.getSerialNum(), spatial.getDeviceVersion()))
    print("|------------|----------------------------------|--------------|------------|")
    print("Number of Acceleration Axes: %i" % (spatial.getAccelerationAxisCount()))
    print("Number of Gyro Axes: %i" % (spatial.getGyroAxisCount()))
    print("Number of Compass Axes: %i" % (spatial.getCompassAxisCount()))

#Event Handler Callback Functions
def SpatialAttached(e):
    attached = e.device
    print("Spatial %i Attached!" % (attached.getSerialNum()))

def SpatialDetached(e):
    detached = e.device
    print("Spatial %i Detached!" % (detached.getSerialNum()))

def SpatialError(e):
    try:
        source = e.device
        print("Spatial %i: Phidget Error %i: %s" % (source.getSerialNum(), e.eCode, e.description))
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))

    

#Main Program Code
try:
    spatial.setOnAttachHandler(SpatialAttached)
    spatial.setOnDetachHandler(SpatialDetached)
    spatial.setOnErrorhandler(SpatialError)
    #spatial.setOnSpatialDataHandler(SpatialData)
except PhidgetException as e:
    print("Phidget Exception %i: %s" % (e.code, e.details))
    print("Exiting....")
    sys.exit(1)

print("Opening phidget object....")

try:
    spatial.openPhidget(302229)        #354120)  #302229)
except PhidgetException as e:
    print("Phidget Exception %i: %s" % (e.code, e.details))
    print("Exiting....")
    sys.exit(1)

print("Waiting for attach....")

try:
    spatial.waitForAttach(50000)
except PhidgetException as e:
    print("Phidget Exception %i: %s" % (e.code, e.details))
    try:
        spatial.closePhidget()
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Exiting....")
        sys.exit(1)
    print("Exiting....")
    sys.exit(1)
else:
    spatial.setDataRate(4)
    DisplayDeviceInfo()


print("Reset calibration constants and set new constants")
try:
    spatial.resetCompassCorrectionParameters()
    # From Coop Report Ben Krupicz (UBC) Sept 2014, Table 1 + appendix C
    # Parameters: magField = 1, offset0, offset1, offset2, A0' = Gain0 * A0, A4' = Gain1 * A4, A8' = Gain2 * A8, A1' = Gain0 * A1, 
    #             A2' = Gain0 * A2, A3' = Gain1 * A3, A5' = Gain1 * A5, A6' = Gain2 * A6, A7' = Gain2 * A7
    # SetParameters(magField, offset0, offset1, offset2, A0', A4', A8', A1', A2', A3', A5', A6', A7')
    if spatial.getSerialNum() == 301816:
        #spatial.setCompassCorrectionParameters(1, 0.0084, 0.1937, 0.0517,
        #                                       0.8877*0.9617, 0.8991*0.9395, 1.0452*1.0988,
        #                                       0.8877*-0.0067, 0.8877*-0.0133, 
        #                                       0.8991*-0.0066, 0.8991*-0.00033789,
        #                                       1.0452*-0.0152, 1.0452*-0.00034909)
        # M.Walters' recalibration (errors under discussion)
        spatial.setCompassCorrectionParameters(1, 0.0247, 0.1831, -0.0506,
                                               1.6537, 1.7979, 1.9713,
                                               -0.0102, -0.0107, 
                                               -0.0110, -0.0044,
                                               -0.0127, -0.0048)
        print "Calibrated phidget %i" %spatial.getSerialNum()
    elif spatial.getSerialNum() == 302229:
        #spatial.setCompassCorrectionParameters(1, 0.0394, 0.2194, -0.2528,
        #                                       0.0633*0.9723, 0.8860*0.9388, 0.9329*1.0880,
        #                                       0.0633*-0.0078, 0.0633*-0.0110,
        #                                       0.8860*-0.0075, 0.8860*0.00035813,
        #                                       0.9329*0.0122, 0.9329*0.00035841)
        spatial.setCompassCorrectionParameters(1, 0.0535, 0.2105, -0.2624,
                                               1.7464, 1.6831, 1.9148,
                                               -0.0151, -0.0107, 
                                               -0.0144, -0.0019,
                                               -0.0116, -0.0021)
        
        print "Calibrated phidget %i" %spatial.getSerialNum()
        # phidget01
    elif spatial.getSerialNum() == 354120:
        spatial.setCompassCorrectionParameters(1, 0.1042, 0.1441, -0.0122,
                                               0.9595, 3.2027, 1.7343,
                                               0.0037, -0.0160, 
                                               0.0123, 0.0032,
                                               -0.0288, 0.0017)
        
        print "Calibrated phidget %i" %spatial.getSerialNum()
    elif spatial.getSerialNum() == 417546:
	spatial.setCompassCorrectionParameters(1, -0.1298, 0.04267, 0.0032,
					       1.8495, 1.8016, 2.1559,
					       -0.0113, -0.0319,
					       -0.0109, -0.0025, 
					       -0.0371, -0.0029)
	print "Calibrated phidget %i" %spatial.getSerialNum()
    elif spatial.getSerialNum() == 415247:
        spatial.setCompassCorrectionParameters(1, -0.1268, 0.0302, -0.0637, 
                                               1.8878, 1.8713, 2.1995, 
                                               -0.0253, -0.2667, 
                                               -0.0251, -0.0056, 
                                               -0.0310, -0.0065)
        print "Calibrated phidget %i" %spatial.getSerialNum()
    elif spatial.getSerialNum() == 414499:
        spatial.setCompassCorrectionParameters(1, -0.1292, 0.0274, -0.0577, 
                                               1.8517, 1.8576, 2.1813, 
                                               -0.0213, -0.0146, 
                                               -0.0214, -0.0047, 
                                               -0.0172, -0.0055)
        print "Calibrated phidget %i" %spatial.getSerialNum()
    elif spatial.getSerialNum() == 414013:
        spatial.setCompassCorrectionParameters(1, -0.1121, 0.0366, -0.0086, 
                                               1.9902, 1.9656, 2.2199, 
                                               -0.0204, -0.0281, 
                                               -0.0203, -0.0023, 
                                               -0.0315, -0.0025)
        print "Calibrated phidget %i" %spatial.getSerialNum()
    elif spatial.getSerialNum() == 417702:
        spatial.setCompassCorrectionParameters(1, -0.1264, 0.0196, -0.0681, 
                                               1.9819, 1.9604, 2.1979, 
                                               -0.0243, -0.0205, 
                                               -0.0241, -0.0075, 
                                               -0.0228, -0.0085)
        print "Calibrated phidget %i" %spatial.getSerialNum()

    elif spatial.getSerialNum() == 354856:
        print "Not yet Calibrated phidget %i" %spatial.getSerialNum()
    elif spatial.getSerialNum() == 354857 :
        print "Not yet Calibrated phidget %i" %spatial.getSerialNum()

    else:
        print("Unknown device, no calibration constants")
        sys.exit(1)
except PhidgetException as e:
    print("Phidget Exception %i: %s" % (e.code, e.details))
    print("Exiting....")
    sys.exit(1)

print("Closing...")

try:
    spatial.closePhidget()
except PhidgetException as e:
    print("Phidget Exception %i: %s" % (e.code, e.details))
    print("Exiting....")
    sys.exit(1)

print("Done.")
sys.exit(0)
