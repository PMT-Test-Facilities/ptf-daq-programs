#!/bin/bash

# Execute at the end of the run to log the run info
# and convert the midas files to rootfiles automatically
echo "exp:  ${MIDAS_EXPT_NAME}"

# Get from odb
# awk magic: -F "  +" : multiples of spaces as delimiter
Run_number=`odb -c 'ls "/Runinfo/Run number"' | awk -F "  +" '{print $2}'`
Start_time=`odb -c 'ls "/Runinfo/Start Time"' | awk -F "   +" '{print $2}'`
Stop_time=`odb -c 'ls "/Runinfo/Stop Time"' | awk -F "   +" '{print $2}'`
Pedestal=`odb -c 'ls "/Experiment/Edit on start/Pedestals run"' | awk -F "  +" '{print $2}'`
Light_source=`odb -c 'ls "/Experiment/Edit on start/Light source"' | awk -F "  +" '{print $2}'`
Laser_power=`odb -c 'ls "/Experiment/Edit on start/Laser power"' | awk -F "  +" '{print $2}'`
Attenuation=`odb -c 'ls "/Experiment/Edit on start/Attenuation (dB)"' | awk -F "  +" '{print $2}'`
Comments=`odb -c 'ls "/Experiment/Edit on start/Comments"' | awk -F "  +" '{print $2}'`
Make_tree=`odb -c 'ls "/Experiment/Edit on start/Make root tree"' | awk -F "  +" '{print $2}'`
Pulser_freq=`odb -c 'ls "/Experiment/Edit on start/Pulser frequency(Hz)"' | awk -F "  +" '{print $2}'`

# Store in file
#printf "${Run_number}\t${Start_time}\t${Stop_time}\t${Pedestal}\t\t${Light_source}\t\t${Laser_power}\t\t${Attenuation}\t\t${Comments}\n" >> ${HOME}/online/custom/runs.txt
printf "${Run_number}\t${Start_time}\t${Stop_time}\t${Pedestal}\t\t${Light_source}\t\t${Laser_power}\t\t${Attenuation}\t\t${Pulser_freq}\t\t${Comments}\n" >> ${HOME}/online/custom/runs.txt

# Make a pretty html table
${HOME}/online/custom/make_runlist.py

# Convert midas to root
if [ ${Make_tree} == "y" ]
then
    ${HOME}/packages/rootana/libAnalyzer/run_convert_to_root.sh ${Run_number} 
    #if [ ${Run_number} -lt 1000 ]
    #then
    #	${HOME}/packages/rootana/libAnalyzer/convert_to_root ${HOME}/online/data/run00${Run_number}sub000.mid.gz >> out_${Run_number}.log 
    #else
    #	${HOME}/packages/rootana/libAnalyzer/convert_to_root ${HOME}/online/data/run0${Run_number}sub000.mid.gz >> out_${Run_number}.log 
    #fi
fi

# Everything runs in $HOME/online
strRun=$(printf "%05d" ${Run_number})
mv out_run${strRun}.root ${HOME}/online/rootfiles
mv out_${strRun}.log ${HOME}/online/logfiles
