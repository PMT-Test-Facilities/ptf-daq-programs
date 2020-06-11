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
"[3] 1",\
"[4] 2",\
"[5] 2",\
"[6] 2",\
"[7] 2",\
"MotorP = INT[8] :",\
"[0] 0",\
"[1] 0",\
"[2] 0",\
"[3] 0",\
"[4] 0",\
"[5] 0",\
"[6] 0",\
"[7] 0",\
"MotorI = INT[8] :",\
"[0] 0",\
"[1] 0",\
"[2] 0",\
"[3] 0",\
"[4] 0",\
"[5] 0",\
"[6] 0",\
"[7] 0",\
"MotorD = INT[8] :",\
"[0] 2500",\
"[1] 0",\
"[2] 0",\
"[3] 0",\
"[4] 0",\
"[5] 0",\
"[6] 0",\
"[7] 0",\
"MotorER = INT[8] :",\
"[0] 0",\
"[1] 0",\
"[2] 0",\
"[3] 0",\
"[4] 0",\
"[5] 0",\
"[6] 0",\
"[7] 0",\
"MotorMO = INT[8] :",\
"[0] 0",\
"[1] 0",\
"[2] 0",\
"[3] 0",\
"[4] 0",\
"[5] 0",\
"[6] 0",\
"[7] 0",\
"Velocity = FLOAT[8] :",\
"[0] 10000",\
"[1] 0",\
"[2] 10000",\
"[3] 2",\
"[4] 200",\
"[5] 0",\
"[6] 0",\
"[7] 0",\
"Acceleration = FLOAT[8] :",\
"[0] 20000",\
"[1] 0",\
"[2] 20000",\
"[3] 0",\
"[4] 20",\
"[5] 0",\
"[6] 0",\
"[7] 0",\
"Deceleration = FLOAT[8] :",\
"[0] 20000",\
"[1] 100",\
"[2] 20000",\
"[3] 100",\
"[4] 20",\
"[5] 100",\
"[6] 100",\
"[7] 100",\
"Jog Velocity = FLOAT[8] :",\
"[0] 500",\
"[1] 2",\
"[2] 500",\
"[3] 2",\
"[4] 200",\
"[5] 2",\
"[6] 2",\
"[7] 2",\
"EncoderPosition = FLOAT[8] :",\
"[0] 0",\
"[1] 0",\
"[2] 0",\
"[3] 0",\
"[4] 0",\
"[5] 0",\
"[6] 0",\
"[7] 0",\
"Move = BOOL[8] :",\
"[0] n",\
"[1] n",\
"[2] n",\
"[3] n",\
"[4] n",\
"[5] n",\
"[6] n",\
"[7] n",\
"Advance = BOOL[8] :",\
"[0] n",\
"[1] n",\
"[2] n",\
"[3] n",\
"[4] n",\
"[5] n",\
"[6] n",\
"[7] n",\
"Jog Pos = BOOL[8] :",\
"[0] n",\
"[1] n",\
"[2] n",\
"[3] n",\
"[4] n",\
"[5] n",\
"[6] n",\
"[7] n",\
"Jog Neg = BOOL[8] :",\
"[0] n",\
"[1] n",\
"[2] n",\
"[3] n",\
"[4] n",\
"[5] n",\
"[6] n",\
"[7] n",\
"Home = BOOL[8] :",\
"[0] n",\
"[1] n",\
"[2] n",\
"[3] n",\
"[4] n",\
"[5] n",\
"[6] n",\
"[7] n",\
"PHome = BOOL[8] :",\
"[0] n",\
"[1] n",\
"[2] n",\
"[3] n",\
"[4] n",\
"[5] n",\
"[6] n",\
"[7] n",\
"Bounce = BOOL[8] :",\
"[0] n",\
"[1] n",\
"[2] n",\
"[3] n",\
"[4] n",\
"[5] n",\
"[6] n",\
"[7] n",\
"Stop = BOOL[8] :",\
"[0] n",\
"[1] n",\
"[2] n",\
"[3] n",\
"[4] n",\
"[5] n",\
"[6] n",\
"[7] n",\
"Index Position = BOOL[8] :",\
"[0] n",\
"[1] n",\
"[2] n",\
"[3] n",\
"[4] n",\
"[5] n",\
"[6] n",\
"[7] n",\
"Slope = FLOAT[8] :",\
"[0] -1.0",\
"[1] -1.0",\
"[2] -1.0",\
"[3] -1.0",\
"[4] 1.0",\
"[5] 1.0",\
"[6] 1.0",\
"[7] 1.0",\
"Offset = FLOAT[8] :",\
"[0] 0",\
"[1] 0",\
"[2] 0",\
"[3] 0",\
"[4] 0",\
"[5] 0",\
"[6] 0",\
"[7] 0",\
"Encoder Slope = FLOAT[8] :",\
"[0] 1",\
"[1] 1",\
"[2] 1",\
"[3] 1",\
"[4] 0",\
"[5] 0",\
"[6] 0",\
"[7] 0",\
"Encoder Offset = FLOAT[8] :",\
"[0] 0",\
"[1] 0",\
"[2] 0",\
"[3] 0",\
"[4] 0",\
"[5] 0",\
"[6] 0",\
"[7] 0",\
"PressThresh = FLOAT[8] :",\
"[0] 0",\
"[1] 0",\
"[2] 0",\
"[3] 0",\
"[4] 0",\
"[5] 0",\
"[6] 0",\
"[7] 0",\
"LimitPolarity = INT[8] :",\
"[0] -1",\
"[1] -1",\
"[2] -1",\
"[3] -1",\
"[4] -1",\
"[5] -1",\
"[6] -1",\
"[7] -1",\
"MotorCurrent = INT[8] :",\
"[0] 1",\
"[1] 1",\
"[2] 1",\
"[3] 1",\
"[4] 1",\
"[5] 1",\
"[6] 1",\
"[7] 1",\
"HoldCurrent = INT[8] :",\
"[0] 1",\
"[1] 1",\
"[2] 1",\
"[3] 1",\
"[4] 1",\
"[5] 1",\
"[6] 1",\
"[7] 1",\
"Torque = FLOAT[8] :",\
"[0] 0.3",\
"[1] 0",\
"[2] 5",\
"[3] 0",\
"[4] 0",\
"[5] 0",\
"[6] 0",\
"[7] 0",\
"PowerOn = BOOL[8] :",\
"[0] n",\
"[1] n",\
"[2] n",\
"[3] n",\
"[4] n",\
"[5] n",\
"[6] n",\
"[7] n",\
"DigitalOut1 = BOOL[8] :",\
"[0] y",\
"[1] n",\
"[2] y",\
"[3] n",\
"[4] n",\
"[5] n",\
"[6] n",\
"[7] n",\
"DigitalOut2 = BOOL[8] :",\
"[0] n",\
"[1] n",\
"[2] n",\
"[3] n",\
"[4] n",\
"[5] n",\
"[6] n",\
"[7] n",\
"Names = STRING[8] :",\
"[32] Supply Reel",\
"[32] SpareB",\
"[32] Takeup Reel",\
"[32] SpareD",\
"[32] Gamma Detector",\
"[32] SpareF",\
"[32] SpareG",\
"[32] SpareH",\
"Destination = FLOAT[8] :",\
"[0] 10000",\
"[1] 0",\
"[2] 0",\
"[3] 0",\
"[4] 0",\
"[5] 0",\
"[6] 0",\
"[7] 0",\
"Letter = STRING[8] :",\
"[32] A",\
"[32] B",\
"[32] C",\
"[32] D",\
"[32] E",\
"[32] F",\
"[32] G",\
"[32] H",\
"",\
"[Channels]",\
"Motors = INT : 8",\
"",\
"[Devices/Motors/BD]",\
"Host = STRING : [256] galil.local",\
"Port = INT : 23",\
"Debug = INT : 0",\
"",\
NULL }


#endif
