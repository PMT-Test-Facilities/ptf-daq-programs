# Overview 

Last Updated: March 04, 2024

Repository for all DAQ and Analysis scripts used in the PTF

This repo holds the code which interfaces with the backend and frontend to make all of our PTF work possible.
The code is in limbo somewhere between c++ and c because of different coding styles used over the ~decade+ that this code has been written over; it's 

fe == front end 

## What's here? 

These executables and c++ files 

 - feMove and feMotor control the motion of the gantries and the tilting of the optical box
 - cd_Galil is an interface between feMove/feMotor and the motors that actually control the gantries
 - fePhidget allows us to use the humidity and spatial phidgets to measure the (1) humidity, (2) the temperature, (3) the local magnetic field, and (4) the orientation of the optical box
 - feWiener manages the HV and the LV for the PMTs and the Helmholtz coils. 


## What about these?

 - fedvm: I don't know
 - fesimdaq: I don't know 
 - cd_Galic_encoder: I don't know 
 - degauss /feDegauss: these are, according to some comments, broken. I think they just set the voltage by following the degaussing steps. This code was also implemented [here](https://github.com/PMT-Test-Facilities/PTF-Field-Scanning/tree/main). 


# Starting Things Up

This assumes that you've already started the server and can go to the ptf website.
If it isn't, go run the `start_daq.sh` bash script in `/home/midptf/online/bin`. 

1. Power everything online on the pdu site.
2. Start the `feptfwiener` program. This controls the HV/LV for the coils and the other things.
3. Start the fePhidget01 frontend.
4. Start the feMotor00  and feMotor01 frontends. Start both! Even if you're only using gantry0. 
At the time of writing the code only works (reliably) if both gantries are movable. 

Reinitializing takes a while! Give it at least 3-4 minutes. The displayed positions may show nonsense temporarily while the optical box tilts around and things don't exactly know where the gantries are 

# Common Problems

## The buffer is too small

When you start a frontend script, it might warn you that the buffer for events is too small and it's changing the buffer size to the minimum value. In this case, changing the code won't actually do anything! 
You need to go online to the midptf site and change the buffer for that frontend manually in the ODB tab. For example, for the phidgets you would go to 
```
    / Equipment / Phidget## / Common / 
```
and change the "Write cache size" to a larger value. Otherwise it just won't work (for reasons unknown). 

## CAEN Drivers won't install 

For the CAEN board, if the drivers won't install, and it doesn't make any sense why that's the case... you might have to reinstall the linux kernel. 
Troubleshoot for a while before resorting to this. You'd have to build a new kernel (lower-risk) and then change the grub settings to use the new kernel on boot-up. 

## Motors won't initialize

A couple possible problems
 - Were both gantry motors (00/01) turned on? 
 - You might just need to power it down, push the gantry over, and power it back up again. 
 - 

# Reference

## Axes

The axes go in an order that is different from the axis order 

    0 - Y axis gantry 0
    1 - X axis gantry 0
    2 - Z axis gantry 0 
    3 - rotation gantry 0 ?
    4 - tilt gantry 0 ?

## Current Problems and Things to Do

### What are all of these html files?

I don't know. I don't even think these are used by the frontend 

### Remove Gantry 1 code support 
Remove gantry1 from feMove. It only causes headaches right now since it's totally unused. 
This should in principle only require us to set all the parameters associated with it to zero. 
Similar to how Vincent disabled turning. 

### Is there a wiki? 

No. A wiki would be useful though. 
There are a handful of PDFs made over several years. 
These are largely out of date now, however. 

# Contribution Rules

It is **imperative** that any changes you make are consistently pushed to github with meaningful commit messages. It is equally important that all changes made are **documented**! 
I cannot stress that enough.

Old code that isn't being used anymore? Delete it.
If you've been committing your code, then you risk nothing by deleting it. **Do not commit any code with blocks of commented code!**

Rather than use duplicates of code, use a *method!* 
Do you need multiple versions of a method because you have multiple data types? 
Use a *template.*