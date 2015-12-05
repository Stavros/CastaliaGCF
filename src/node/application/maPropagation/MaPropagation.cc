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

#include "MaPropagation.h"

Define_Module(MaPropagation);

void MaPropagation::startup()
{	
	trace() <<"App started" ;
		
	/*
	locData tmpData;
	topo = new cTopology("topo");
	topo->extractByNedTypeName(cStringTokenizer("node.Node").asVector());
	for(int i=0; i<numNodes; i++)
	{
	tmpData.nodeID = (unsigned short)self;
	trace() << tmpData.nodeID;
	tmpData.locX = mobilityModule->getLocation().x;
	trace() << tmpData.locX;
	tmpData.locY = mobilityModule->getLocation().y;
	trace() << tmpData.locY;	  
	}
	*/
	
	/*
	// get a reference to the instance of the SN module
	//cModule *theSNModule;
	//theSNModule = parentModule()->parentModule();
	theSNModule = getParentModule()->getParentModule();

	cModule *tmpNode;
	nodeLocRec tmpRec;

	// for each Node sumbodule of SN do...
	for(int i=0; i < (int)(theSNModule->par("numNodes")); i++)
	{
		// get the reference to the the instance
	      //  of the Node[i] module
		tmpNode = theSNModule->submodule("node", i);

		// get the location (x,y) of Node[i]
		tmpRec.locX = tmpNode->par("xCoor");
		tmpRec.locY = tmpNode->par("yCoor");

		// store location in nodesLocVec vector
		updateNeighborTablenodesLocVec.push_back(tmpRec);

	}

	// Now, nodesLocVec hold the
	// location (x,y) of each node 	
	*/
	
	
	neighborTable.clear();
	
	//double diff = p[i] - q[i];
	//sum += diff*diff;
      
	//packetsPerNode = par("packetsPerNode");
	tempThreshold = par("tempThreshold");
	totalPackets = 0;
	currMaxReceivedValue = -1.0;
	currMaxSensedValue = -1.0;
	sentOnce = false;
	theValue = 0;
	randomBackoffIntervalFraction = genk_dblrand(0);
	sentOnce = false;
	sampleInterval = ((double)par("sampleInterval")) / 1000.0;
	setTimer(REQUEST_SAMPLE, sampleInterval * randomBackoffIntervalFraction);
	//setTimer(REQUEST_SAMPLE, 0);
	node_xy = check_and_cast<VirtualMobilityManager*>(getParentModule()->getSubmodule("MobilityManager"));
	//newx = (int)node_xy->getLocation().x;
	//newy = (int)node_xy->getLocation().y;
	trace() << "Node's " << SELF_NETWORK_ADDRESS << " X,Y Coordinates are = " << newx << "," << newy;
	//updateNeighborTable(atoi(SELF_NETWORK_ADDRESS), sequenceNumber, x, y);
	totalSNnodes = getParentModule()->getParentModule()->par("numNodes");
	trace() << "Total number of Node is : " << totalSNnodes;
	neighborTable.reserve(totalSNnodes);
	self = getParentModule()->getIndex();
	
}

void MaPropagation::timerFiredCallback(int index)
{
	switch (index) {
		case REQUEST_SAMPLE:{
			requestSensorReading();
			setTimer(REQUEST_SAMPLE, sampleInterval);
			break;
		}
	}
}

void MaPropagation::fromNetworkLayer(ApplicationPacket * rcvPacket, const char *source, double rssi, double lqi)
{
	int sequenceNumber = rcvPacket->getSequenceNumber();
	trace() << "Received packet from node " << source << ", SN = " << sequenceNumber;
	
	double receivedData = rcvPacket->getData();
	trace() << "Received Data : " << receivedData;;
	//toNetworkLayer(createGenericDataPacket(receivedData, 1), BROADCAST_NETWORK_ADDRESS);
	//trace() << "Got the value: " << theValue;
	//updateNeighborTable(atoi(source), rcvPacket->getSequenceNumber());
	newx = (int)node_xy->getLocation().x;
	newy = (int)node_xy->getLocation().y;
	updateNeighborTable(atoi(SELF_NETWORK_ADDRESS), sequenceNumber, newx, newy);
	//getNodeDistance();
	
	totalPackets++;
	if (receivedData > currMaxReceivedValue)
	currMaxReceivedValue = receivedData;

	//if (receivedData > tempThreshold && !sentOnce) {
		//theValue = receivedData;
		//toNetworkLayer(createGenericDataPacket(receivedData, 1), BROADCAST_NETWORK_ADDRESS);
		//sentOnce = true;
		//trace() << "Got the value: " << theValue;
	  if (isSink)
		trace() << "Sink received from: " << source << " value= " << rcvPacket->getData();		
	//}
	
}

void MaPropagation::handleSensorReading(SensorReadingMessage * rcvReading)
{
	double sensedValue = rcvReading->getSensedValue();
	trace() << "Sensed = " << sensedValue;

	if (sensedValue > currMaxSensedValue)
		currMaxSensedValue = sensedValue;

	if (sensedValue > tempThreshold && !sentOnce) {
		theValue = sensedValue;
		//toNetworkLayer(createGenericDataPacket(sensedValue, 1), BROADCAST_NETWORK_ADDRESS);
		toNetworkLayer(createGenericDataPacket(sensedValue, 1), BROADCAST_NETWORK_ADDRESS);
		sentOnce = true;
	}
}

void MaPropagation::finishSpecific()
{
	declareOutput("got value");
	if (theValue > tempThreshold)
		collectOutput("got value", "yes/no", 1);
	else
		collectOutput("got value", "yes/no", 0);
	declareOutput("app packets received");
	collectOutput("app packets received", "", totalPackets);
	//trace() << " Node id: " << tmpData.nodeID;
	
	//for (int i = 0; i < (int)neighborTable.size(); i++) {
	//	collectOutput("Packets received", neighborTable[i].id);
	//}
	for (int i = 0; i < (int)neighborTable.size(); i++) {
		collectOutput("Packets received", neighborTable[i].id,
			      "Success", neighborTable[i].receivedPackets);
	}	
}

void MaPropagation::updateNeighborTable(int nodeID, int theSN, int x_node, int y_node )
{
    int i = 0, pos = -1;
    int tblSize = (int)neighborTable.size();   

    for (i = 0; i < tblSize; i++)
        if (neighborTable[i].id == nodeID){
            pos = i;
	    break;
	}

    if (pos == -1) {
        neighborRecord newRec;
        newRec.id = nodeID;
        newRec.timesRx = 1;
        //newRec.remainingEnergy =(int)getParentModule()->getSubmodule("node",nodeID)->getSubmodule("Application")->par("remainingEnergy") ;
        //trace()<<"New Neighbour energy" << newRec.remainingEnergy;
        //newRec.x = (int)getParentModule()->getParentModule()->getSubmodule("node",nodeID)->getSubmodule("Application")->par("xCoordinate");
        //newRec.y = (int)getParentModule()->getParentModule()->getSubmodule("node",nodeID)->getSubmodule("Application")->par("yCoordinate");
	//x_node = (int)getParentModule()->getLocation().X;
	//y_node = (int)getParentModule()->getLocation().Y;
	newRec.x = x_node;
	newRec.y = y_node;
        trace()<<"New Neighbour coords x,y : " << newRec.x << "," <<newRec.y;   
        //double distance = sqrt (pow ((xCoordinate - newRec.x),2) + pow ((yCoordinate - newRec.y),2) );
	double distance = sqrt (pow ((newRec.x - xCoordinate),2) + pow ((newRec.y - yCoordinate),2) );
	
        newRec.distance =(int) distance;
        trace()<<"Distance from this node is: New Rec   "<< newRec.distance;
        if (distance > maxDistance)
        {
            maxDistance = distance;
            maxDistID = newRec.id;
        }
       

        if ((theSN >= 0) && (theSN < packetsPerNode))
            newRec.receivedPackets = 1;

        neighborTable.push_back(newRec);
	// print the last item
	trace() << "New neighbor for node : " << self << " is node : "<< neighborTable[(int)neighborTable.size()-1].id;

    } else {
      //it's an already known neighbor
	neighborTable[pos].x = x_node; // updating of location
	neighborTable[pos].y = y_node;
        neighborTable[pos].timesRx++;

        if ((theSN >= 0) && (theSN < packetsPerNode))
            neighborTable[pos].receivedPackets++;
    }
    //double s = calculateNodeDensity();
    
    trace() << "Neighbors list of node " << self << ":";
    tblSize = (int)neighborTable.size(); 
    for (int j = 0; j < tblSize; j++){
     //neighborRecord rec= neighborTable[j];
     trace()<< "Distance to node:REC:  "<< neighborTable[j].id << " is "<< neighborTable[j].distance;
    }
     //double dist = neighborTable[j].distance;   
    trace() << "--------------";
}

void MaPropagation::getNodeDistance()
{
   
    int size = (int)neighborTable.size();
     
    for (int j = 0; j < size; j++)
    {

     neighborRecord rec= neighborTable[j];
     trace()<< "Distance to node:REC:  "<< rec.id << " is "<< rec.distance;
         

        //double dist = neighborTable[j].distance;
   
 
    }
   
}

