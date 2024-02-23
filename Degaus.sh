#!/bin/bash

#Coil 1: /Equipment/PtfWiener/Settings/outputSwitch[9]
#Coil 2: /Equipment/PtfWiener/Settings/outputSwitch[8]
#Coil 3: /Equipment/PtfWiener/Settings/outputSwitch[2]
#Coil 4: /Equipment/PtfWiener/Settings/outputSwitch[6]
#Coil 5: /Equipment/PtfWiener/Settings/outputSwitch[1]
#Coil 6: /Equipment/PtfWiener/Settings/outputSwitch[0]

#This code degausses in 8 steps
#I have already calculated the voltage values for each step based on the formula in Reference Manual of PTF.


#It is important to keep the output switches on before running this script


# Set voltages for step 1:
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[9]" 10'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[8]" 13.6'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[2]" 0.295'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[6]" 1.808'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 0.295'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 1.78'


sleep 30

# Set voltages for step 2:
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[9]" 0.636'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[8]" 1.077'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[2]" 0'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[6]" 0'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 0'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 0.045'


sleep 20


# Set voltages for step 3:
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[9]" 3.779'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[8]" 5.062'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[2]" 0.152'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[6]" 0.806'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 0.152'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 0.758'


sleep 15


# Set voltages for step 4:
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[9]" 2.743'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[8]" 3.794'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[2]" 0.073'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[6]" 0.447'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 0.073'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 0.465'


sleep 10


# Set voltages for step 5:
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[9]" 3.085'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[8]" 4.197'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[2]" 0.114'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[6]" 0.607'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 0.114'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 0.585'


sleep 10


# Set voltages for step 6:
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[9]" 2.972'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[8]" 4.069'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[2]" 0.093'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[6]" 0.536'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 0.093'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 0.536'


sleep 10


# Set voltages for step 7:
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[9]" 3.009'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[8]" 4.11'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[2]" 0.104'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[6]" 0.568'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 0.104'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 0.556'


sleep 10


# Set voltages for step 8:
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[9]" 2.997'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[8]" 4.097'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[2]" 0.098'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[6]" 0.554'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 0.098'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 0.548'


sleep 10


# Set voltages for step 9:
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[9]" 3'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[8]" 4.1'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[2]" 0.1'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[6]" 0.558'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[1]" 0.1'
odbedit -c 'set "/Equipment/PtfWiener/Settings/outputVoltage[0]" 0.550'

sleep 5

echo "Done"
