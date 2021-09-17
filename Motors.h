/********************************************************************\

  Name:         Motors.h, based on experim.h_old
  Created by:   ODBedit program

  Contents:     This file contains C structures for the "Experiment"
                tree in the ODB and the "/Analyzer/Parameters" tree.

                Additionally, it contains the "Settings" subtree for
                all items listed under "/Equipment" as well as their
                event definition.

                It can be used by the frontend and analyzer to work
                with these information.

                All C structures are accompanied with a string represen-
                tation which can be used in the db_create_record function
                to setup an ODB structure which matches the C structure.

  Created on:   Tue Sep 07 15:54:33 2004

\********************************************************************/

#ifndef EXCL_MOTORS

#define MOTORS_SETTINGS_DEFINED

#define MOTORS_SETTINGS_STR(_name) char const *_name[] = {\
"[.]",\
"Status_in = INT : 0",\
"Status_invar = INT : 0",\
"MotorType = FLOAT[8] :",\
"[0] 1",\
"[1] 1",\
"[2] 1",\
"MotorP = INT[8] :",\
"[0] 0",\
"[1] 0",\
"[2] 0",\
"MotorI = INT[8] :",\
"[0] 0",\
"[1] 0",\
"[2] 0",\
"MotorD = INT[8] :",\
"[0] 2500",\
"[1] 0",\
"[2] 0",\
"MotorER = INT[8] :",\
"[0] 0",\
"[1] 0",\
"[2] 0",\
"MotorMO = INT[8] :",\
"[0] 0",\
"[1] 0",\
"[2] 0",\
"Velocity = FLOAT[8] :",\
"[0] 10000",\
"[1] 0",\
"[2] 10000",\
"Acceleration = FLOAT[8] :",\
"[0] 20000",\
"[1] 0",\
"[2] 20000",\
"Deceleration = FLOAT[8] :",\
"[0] 20000",\
"[1] 100",\
"[2] 20000",\
"Jog Velocity = FLOAT[8] :",\
"[0] 500",\
"[1] 2",\
"[2] 500",\
"EncoderPosition = FLOAT[8] :",\
"[0] 0",\
"[1] 0",\
"[2] 0",\
"Move = BOOL[8] :",\
"[0] n",\
"[1] n",\
"[2] n",\
"Advance = BOOL[8] :",\
"[0] n",\
"[1] n",\
"[2] n",\
"Jog Pos = BOOL[8] :",\
"[0] n",\
"[1] n",\
"[2] n",\
"Jog Neg = BOOL[8] :",\
"[0] n",\
"[1] n",\
"[2] n",\
"Home = BOOL[8] :",\
"[0] n",\
"[1] n",\
"[2] n",\
"PHome = BOOL[8] :",\
"[0] n",\
"[1] n",\
"[2] n",\
"Bounce = BOOL[8] :",\
"[0] n",\
"[1] n",\
"[2] n",\
"Stop = BOOL[8] :",\
"[0] n",\
"[1] n",\
"[2] n",\
"Index Position = BOOL[8] :",\
"[0] n",\
"[1] n",\
"[2] n",\
"Slope = FLOAT[8] :",\
"[0] -1.0",\
"[1] -1.0",\
"[2] -1.0",\
"Offset = FLOAT[8] :",\
"[0] 0",\
"[1] 0",\
"[2] 0",\
"Encoder Slope = FLOAT[8] :",\
"[0] 1",\
"[1] 1",\
"[2] 1",\
"Encoder Offset = FLOAT[8] :",\
"[0] 0",\
"[1] 0",\
"[2] 0",\
"PressThresh = FLOAT[8] :",\
"[0] 0",\
"[1] 0",\
"[2] 0",\
"LimitPolarity = INT[8] :",\
"[0] -1",\
"[1] -1",\
"[2] -1",\
"MotorCurrent = INT[8] :",\
"[0] 1",\
"[1] 1",\
"[2] 1",\
"HoldCurrent = INT[8] :",\
"[0] 1",\
"[1] 1",\
"[2] 1",\
"Torque = FLOAT[8] :",\
"[0] 0.3",\
"[1] 0",\
"[2] 5",\
"PowerOn = BOOL[8] :",\
"[0] n",\
"[1] n",\
"[2] n",\
"DigitalOut1 = BOOL[8] :",\
"[0] y",\
"[1] n",\
"[2] y",\
"DigitalOut2 = BOOL[8] :",\
"[0] n",\
"[1] n",\
"[2] n",\
"Names = STRING[8] :",\
"[32] Supply Reel",\
"[32] SpareB",\
"[32] Takeup Reel",\
"Destination = FLOAT[8] :",\
"[0] 10000",\
"[1] 0",\
"[2] 0",\
"Letter = STRING[8] :",\
"[32] A",\
"[32] B",\
"[32] C",\
"",\
"[Channels]",\
"Motors = INT : 3",\
"",\
"[Devices/Motors/BD]",\
"Host = STRING : [256] 192.168.0.100",\
"Port = INT : 23",\
"Debug = INT : 0",\
"",\
NULL }


#endif
