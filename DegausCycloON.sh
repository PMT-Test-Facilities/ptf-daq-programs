#!/bin/bash

# Switch on voltage channels for desired channel, eg. Coils 1 & 2:
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputSwitch[8]" 1'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputSwitch[9]" 1'

# If ON:
# Convert desired current to current_user +- X mA and convert to V 

# Set voltages part 1:
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[8]" 6.46'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[9]" 9.22'
sleep 5

#Degauss in x (coils 3 & 4) for ASYMMETRIC CURRENTS
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[2]" 0.35'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[4]" 0'
sleep 5
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[2]" 2.80'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[4]" 2.54'
sleep 5
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[2]" 1.52'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[4]" 1.24'
sleep 5 
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[2]" 2.19'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[4]" 1.91'
sleep 5
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[2]" 1.84'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[4]" 1.56'
sleep 5
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[2]" 2.02'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[4]" 1.75'
sleep 5
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[2]" 1.93'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[4]" 1.66'
sleep 5
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[2]" 1.96'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[4]" 1.69'

#Degauss in y (coils 5 & 6) for ASYMMETRIC CURRENTS
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 0.69'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 0'
sleep 5
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 2.8'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 2.01'
sleep 5
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 1.30'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 0.63'
sleep 5
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 2.30'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 1.61'
sleep 5
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 1.60'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 0.93'
sleep 5
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 2.15'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 1.42'
sleep 5
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 1.78'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 1.08'
sleep 5
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 2.06'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 1.32'
sleep 5
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 1.98'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 1.22'

#Degauss in x (coils 3 & 4) for symmetric currents
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[2]" 0'
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[4]" 0'
#sleep 2
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[2]" 2.68'
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[4]" 2.75'
#sleep 2
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[2]" 1.34'
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[4]" 1.39'
#sleep 2 
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[2]" 2.02'
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[4]" 2.09'
#sleep 2
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[2]" 1.69'
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[4]" 1.75'
#sleep 2
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[2]" 1.83'
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[4]" 1.89'
#sleep 2
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[2]" 1.74'
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[4]" 1.80'
#sleep 2
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[2]" 1.78'
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[4]" 1.83'

#Degauss in y (coils 5 & 6) for symmetric currents
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 0'
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 0'
#sleep 2
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 3.12'
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 3.00'
#sleep 2
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 0.69'
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 0.53'
#sleep 2
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 2.29'
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 2.04'
#sleep 2
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 1.21'
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 1.03'
#sleep 2
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 1.98'
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 1.73'
#sleep 2
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 1.56'
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 1.34'
#sleep 2
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 1.85'
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 1.60'
#sleep 2
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 1.74'
#odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 1.52'

echo "Done"
