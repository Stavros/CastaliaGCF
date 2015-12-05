/*********************************************************/
/*  Copyright (c) 2011. University of Pau, France        */
/*  LIUPPA Laboratory, T2I Team                          */
/*                                                       */
/*  Permission to use, copy, modify and distribute this  */
/*  code, without fee, and without written agreement is  */
/*  hereby granted, provided that the above copyright    */
/*  notice and the authors appear in all copies          */
/*                                                       */
/*  GCF Routing Protocol                                 */
/*  Version:  1.0                                        */
/*  Author: Stavros Kalapothas <stavros@ubinet.gr>       */
/*														 */
/*  Based on:											 */
/*  GPSR Routing Protocol                                */
/*  Authors: Diop Mamour <serignemamour.diop@gmail.com>  */
/*           Congduc Pham <congduc.pham@univ-pau.fr>     */
/*********************************************************/


#ifndef _GPSRROUTING_H_
#define _GPSRROUTING_H_

#include <map>
#include "VirtualRouting.h"
#include "GpsrRoutingControl_m.h"
#include "GpsrRoutingPacket_m.h"
#include <stdio.h>
#include <limits.h>
#include <math.h>
#include <list>
//#include <iterator>     // std::next
//#include <algorithm>
//#include <vector>

#define DEFAULT_GPSR_TIMEOUT   200.0   
        /* the default time out period is 200.0 sec
           If the hello messge was not received during this period
           the entry in the neighbor list may be deleted 
        */

// if something is wrong, will retry sending HELLO message GPSR_RETRY_HELLO_DELAY second later
#define GPSR_RETRY_HELLO_DELAY 1

// Number of vertices in the graph
#define V 16

using namespace std;

struct GPSR_neighborRecord {
	int id;      // the node's ID
	int x;       // the node's coordinates : the geographic information
	int y;
	double ts;   //the last time stamp of the hello msg from it
 	int timesRx;   

	GPSR_neighborRecord() {
	   id = 0;
       x = 0.0;
       y = 0.0;
       ts = 0.0;
	   timesRx = 0;
    }
    
};

struct sink {
       int id;          // the Sink's ID
	   int x;       // the Sink's coordonates : the geographic information
       int y;
};

enum GpsrRoutingTimers {
    GPSR_HELLO_MSG_REFRESH_TIMER = 0,
    GPSR_HELLO_MSG_EXPIRE_TIMER  = 1,
};

class GpsrRouting: public VirtualRouting {
 private:
    // Parameters
    int GpsrSetupFrameOverhead;	// in bytes
    double netSetupTimeout;
    bool collectTraceInfo;
    int currentSequenceNumber;
    double helloInterval;
    double activeRouteTimeout; //in s

    // GpsrRouting-related member variables
    int self;         // the node's ID
    double self_xCoo; // store the node's position in meters
    double self_yCoo;
	bool isCoordinateSet; // to know whether the node's position has been set or not
    int totalSNnodes;
    int packetsPerNode;
    bool isSink;		//is a .ned file parameter of the Application module
    sink mySink; 
    int seqHello;
	vector<GPSR_neighborRecord> neighborTable;
        //VirtualMobilityManager *nodeMobilityModule;
	vector<double> G[10000];
	//list<pair<int,int> > path;
    list <int> pathd[16][16];  	 // a 2d array of lists that contain all path's distance
	//vector < list<int> > *path2;
	bool comp;

 protected:

    void startup();
    void finishSpecific();
    void fromApplicationLayer(cPacket *, const char *);
    void fromMacLayer(cPacket *, int, double, double);
	void handleNetworkControlCommand(cMessage *);
    void sendTopologySetupPacket();
    void timerFiredCallback(int);
    void processBufferedPacket();

    void sendHelloMessage();
    void processDataPacketFromMacLayer(GpsrPacket*);

    void updateNeighborTable(int, int, int, int); // add a possible new neighbor
    int getNextHopGreedy(int, int);              // Greedy forwarding mode
    int getNextHopEucl(int, int);		  // Euclidean Distance from Self *** 150614
    //int getNextHopGCF(list <int> paths[]);		// GCF implementation *** 220714
    int getNextHopGCF(int);		// GCF implementation *** 220714
    int getNextHopPerimeter(int, int);          // Perimeter mode
    double distance(int, int, int, int); //calculate distance between 2 nodes
    void distTable (int, int, int, int);
    void dijkstra(double distarArr[V][V], int src);
    //int minDistance(int dist[], bool sptSet[]);
    void printSolution(int dist[], int parent[], int n);
    //void printPath(int dest);
    //int minDistance(int, bool);
    //int printSolution(int, int);
 //public:
    //nodeModule cModule;
    //int getParentModule();
    public:
    int srcX;
    int srcY;
    int curr;
    double powrArr[16][16];
    list <int> path[16][16];   	  // a 2d array of lists that contain all path's nodes
    list <int> paths[16];		// an array of lists that contains distances sorted
    list <int> visit[16][16];			// a list of nodes visited
    int flag[1];			// a array of nodes visited index flag
    int flags;
    int pathf;
    //int flag;

};

#endif				//GPSRROUTINGMODULE
