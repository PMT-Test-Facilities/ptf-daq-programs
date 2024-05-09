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
 - field-setting code is kept elsewhere, see ~/online/scripts/calibrate_coils. There's an feButtonManager frontend that runs the front end interface for setting the magnetic field. 


## What about these?

 - fedvm: I don't know
 - fesimdaq: I don't know 
 - cd_Galic_encoder: I don't know 
 - degauss /feDegauss: these are, according to some comments, broken. I think they just set the voltage by following the degaussing steps. This code was also re-implemented [here](https://github.com/PMT-Test-Facilities/PTF-Field-Scanning/tree/main). 


# Starting Things Up

This assumes that you've already started the server and can go to the ptf website.
If it isn't, go run the `start_daq.sh` bash script in `/home/midptf/online/bin`. 

1. Power the VME Crate, the NIM Crate, the Wiener Crate, and both gantries up. 
2. Start the `feptfwiener` program. This controls the HV/LV for the coils and the other things.
3. Start the fePhidget00 frontend.
4. Start the feMotor00  and feMotor01 frontends. Start both! Even if you're only using gantry0. 
At the time of writing the code only works (reliably) if both gantries are movable. 
5. Start feMove, and then in the movement page reinitialize the gantries. This will take a while, and you will see several error messages. This is normal! 
6. If you'll be doing a scan, navigate to the HV page. Press Main Switch ON, Output ON, Action-On the 20" PMT, and Action-On the G0-Monitor PMt. These will take a while to ramp up. 
7. Wait until the position is initialized and both PMTs reach their HV. 

Reinitializing takes a while! Give it at least 3-4 minutes. The displayed positions may show nonsense temporarily while the optical box tilts around and things don't exactly know where the gantries are 

# Common Problems

Need to restart things? 
```
sudo reboot -h now 
```
You should make sure nobody else is using this before rebooting.

## Crashes

### System Crash on Scan Start

This is something I (Ben S) saw once as of writing (Apr 26, 2024). 
Everything would seem fine during the preparations, but then when the run was started, the whole midas frontend would crash. 
Midas would become totally unrecoverable until the DAQ PC was restarted. 

**Solution**: reboot everything. Then get the frontend running again and use the frontend to ramp down the voltage for the PMTs. Then power everything down and shut down thefront end programs. Treat the database as if its full. Delete unused keys. Dump the ODB to a json file, clean the whole thing out, and reload the data from the json file. This worked for me, hopefully it'll work for you too. 



## The rotation is inaccurate 

For starters, yuo might just have to recalibrate these. 
Neither one has an absolute, measured, scaling. 
Instead, pre-stored stepper motor velocities and steps-to-degrees values are used to convert between stepper motor counts and orientations in physical space. 

These values occasionally get out of sync with reality. 

The current way of calibrating these is still under development. 

### Tilt

A few common problems
 - the belt is loose. The belt needs to be surprisingly tight, any slack will throw it off. The phidget is only used initially to calculate the parameters in the function that maps motor step values to optical box angle. 
 - Can't reinitialize - tilt1 diagreement between ODB and phidget! This can happen if you accidentally turn the phidget on for the i=1 frontend. Just manually set `/Equipment/Phidget01/Variables/PH01[7]` to 0.0. 

### Yaw 


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
 - The fePhidget00 frontend was not started. This can *seriously* mess things up! It will keep twisting around and around until something gets jammed. 

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

# Todo


# Contribution Rules

## Coding Standards

### Commit Your Code

It is **imperative** that any changes you make are consistently pushed to github with meaningful commit messages. It is equally important that all changes made are **documented**! 
At least as far as docstrings and comments about what functions are supposed to do!

### Use Git

**DO NOT DELETE AND RE-UPLOAD FILES.** I cannot stress this enough. **DO NOT DELETE AND RE-UPLOAD FILES.** Any commit you do like this will be reverted! Any pull request denied! 
If you aren't sure how to make a git commit, please ask. 
**DO NOT DELETE AND RE-UPLOAD FILES.**
By deleting and re-uploading a file, you remove the history of the changes to the file. 
This defeats the entire purpose of git!!! This makes debugging the effects of changes incredibly difficult and time consuming.

### Keep things clean

Old code that isn't being used anymore? Delete it.
If you've been committing your code, then you risk nothing by deleting it. **Do not commit any code with blocks of commented code!**

### Avoid the pre-processor

** Try to restrict preprocessor definitions to if/else blocks.** These obfuscate code in a nasty way where what you see is not what the compiler sees. 
These are all throughout the code, and I'd like to minimize their useage over time.
Pre-processor definitions also avoid the strongly typed value definitions that are at the very core of c++. 
There's a much greater chance of overlap in naming and there's no guarantee a ``constant" defined through these statements stay constant. 

### Try to make things general

Rather than use duplicates of code, use a method.
Do you need multiple versions of a method because you have multiple data types? 
Use a *template.*
If you don't know what those things are, either ask me for help, or Google it! 
There are a lot of c++ resources available online. 
These will make your code much easier to read and much easier to understand. 
