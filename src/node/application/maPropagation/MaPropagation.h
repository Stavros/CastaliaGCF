/*******************************************************************************
 *  Copyright: National ICT Australia,  2007 - 2011                            *
 *  Developed at the ATP lab, Networked Systems research theme                 *
 *  Author(s): Athanassios Boulis, Dimosthenis Pediaditakis, Yuriy Tselishchev *
 *  This file is distributed under the terms in the attached LICENSE file.     *
 *  If you do not find this file, copies can be found by writing to:           *
 *                                                                             *
 *      NICTA, Locked Bag 9013, Alexandria, NSW 1435, Australia                *
 *      Attention:  License Inquiry.                                           *
 *                                                                             *
 *******************************************************************************/

#ifndef _MAPROPAGATION_H_
#define _MAPROPAGATION_H_

#include "VirtualApplication.h"
//#include <vector> 

using namespace std;

enum MaPropagationTimers {
	REQUEST_SAMPLE = 1,
};

struct neighborRecord {
	int id;
	int timesRx;
	int remainingEnergy;
	int distance;
	int x;
	int y;
	int receivedPackets;
};

struct nodeLocRec
{
        double locX;
        double locY;

};


struct locData {
	unsigned short nodeID;	//the ID of the Node
	double locX;			// x-coordinate of the node
	double locY;			// y-coordinate of the node
};

class MaPropagation: public VirtualApplication {
 private:
	//double distEuclidean [4][4];
	//double powrEuclidean [4][4];
	double geox;
	double geoy;
	double diffx;
	double diffy;
	double dist;
	//int pwr1, pwr2, pwr3;
	vector<neighborRecord> neighborTable;
	vector <nodeLocRec> nodesLocVec;
	vector <locData> locDataArr;
	int totalPackets;
	int sentOnce;
	double theValue;
	double tempThreshold;
	vector<double> sensedValues;
	int packetsSent;
	int serialNumber;
	int totalSNnodes;
	double txInterval_perNode;
	double txInterval_total;
	int packetsPerNode;
	int tmpRec;
	int numNodes;
	double currMaxReceivedValue;
	double currMaxSensedValue;
	double sampleInterval;
	
	//cModule *theSNModule; 
	//cModule *tmpNode; 
	//cTopology *topo;
	double maxDistance;
	int maxDistID;
	int xCoordinate;
	int yCoordinate;
	double randomBackoffIntervalFraction;
	int newx;
	int newy;
	VirtualMobilityManager *node_xy;
	int self;         // the node's ID

 protected:
	void startup();
	void finishSpecific();
	void fromNetworkLayer(ApplicationPacket *, const char *, double, double);
	void handleSensorReading(SensorReadingMessage *);
	void timerFiredCallback(int);
	//void calcEuclidean(int node1, int node2);
	void updateNeighborTable(int, int, int, int);
	//void powrEuclidean(int node1, int node2, int pwr);
	void djikstra();
	void getNodeDistance();

};

#endif				// _MAPROPAGATION_APPLICATIONMODULE_H_
