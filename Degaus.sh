#!/bin/bash

# Switch on voltage channels for desired channel, eg. Coils 1 & 2:
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputSwitch[8]" 1'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputSwitch[9]" 1'

# If ON:
# Convert desired current to current_user +- X mA and convert to V 

# Set voltages part 1:
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[8]" 1'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[9]" 6.76'
sleep 2

#Degauss in x (coils 3 & 4)
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[2]" 0.69'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[4]" 0'
sleep 2
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[2]" 3.51'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[4]" 2.91'
sleep 2
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[2]" 1.47'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[4]" 0.85'
sleep 2 
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[2]" 2.62'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[4]" 2.01'
sleep 2
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[2]" 2.20'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[4]" 1.58'
sleep 2
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[2]" 2.45'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[4]" 1.85'
sleep 2
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[2]" 2.32'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[4]" 1.71'
sleep 2
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[2]" 0'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[4]" 0'

#Degauss in y (coils 5 & 6)
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 0'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 0'
sleep 2
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 3.2'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 3.0'
sleep 2
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 0.71'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 0.56'
sleep 2
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 2.3'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 2.11'
sleep 2
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 1.26'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 1.11'
sleep 2
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 1.96'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 1.81'
sleep 2
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 1.55'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 1.41'
sleep 2
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 1.86'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 1.69'
sleep 2
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 0'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 0'

echo "Done"
