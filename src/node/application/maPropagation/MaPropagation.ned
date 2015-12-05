//***************************************************************************************
//*  Copyright: National ICT Australia, 2007 - 2010                                     *
//*  Developed at the ATP lab, Networked Systems research theme                         *
//*  Author(s): Athanassios Boulis, Dimosthenis Pediaditakis                            *
//*  This file is distributed under the terms in the attached LICENSE file.             *
//*  If you do not find this file, copies can be found by writing to:                   *
//*                                                                                     *
//*      NICTA, Locked Bag 9013, Alexandria, NSW 1435, Australia                        *
//*      Attention:  License Inquiry.                                                   *
//*                                                                                     *
//***************************************************************************************

package node.application.maPropagation;

// The sensor node module. Connects to the wireless channel in order to communicate 
// with other nodes. Connects to psysical processes so it can sample them.

simple MaPropagation like node.application.iApplication {
 parameters:
 	string applicationID = default ("maPropagation");
	bool collectTraceInfo = default (true);
	int priority = default (1);
	int packetHeaderOverhead = default (2);	// in bytes
	int constantDataPayload = default (2);	// in bytes
	int tempThreshold = default(1);
	int sampleInterval = default (60000);
	//double directNodeValueAssignment = default ("(11) 0:40 1:33 2:20 3:44 4:15 5:11 6:19 7:29");
	//string directNodeValueAssignment = default ("(19) 0:40 1:33 2:20 3:44 4:15 5:11 6:19 7:29");
	bool isSink = default (false);
	
	int packetSpacing = default (100);	// in ms
	int packetsPerNode = default (100);
	int packetSize = default (32);	// in bytes	
	
	int xCoordinate = default (30);
	int yCoordinate = default (0);
	
 gates:
 	output toCommunicationModule;
	output toSensorDeviceManager;
	input fromCommunicationModule;
	input fromSensorDeviceManager;
	input fromResourceManager;
}

