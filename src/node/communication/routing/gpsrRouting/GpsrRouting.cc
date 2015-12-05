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

#include "GpsrRouting.h"

Define_Module(GpsrRouting);

//================================================================
//    startup
//================================================================
void GpsrRouting::startup(){

    self = getParentModule()->getParentModule()->getIndex();
	isCoordinateSet = false;    

    totalSNnodes = getParentModule()->getParentModule()->getParentModule()->par("numNodes");
    
    //trace() << "Total node = " << totalSNnodes << endl;
    
    //helloInterval = (double)par("helloInterval") / 1000.0;
    helloInterval = (double)par("helloInterval") / 1000.0;
    activeRouteTimeout = (double)par("activeRouteTimeout") / 1000.0;
    packetsPerNode = par("packetsPerNode");
    neighborTable.clear();
    neighborTable.reserve(totalSNnodes);
	seqHello = par("seqHello");
	pathf = 1;
	//int flag;
	//visit.push_back(1);
    
    // check that the Application module used has the boolean parameter "isSink"
    cModule *appModule = getParentModule()->getParentModule()->getSubmodule("Application");
    //cModule *nodeModule = getParentModule()->getParentModule("node");
    //nodeModule = check_and_cast<VirtualMobilityManager*>(getParentModule()->getSubmodule("MobilityManager"));
    //self = getParentModule()->getIndex();
    
    if (appModule->hasPar("isSink"))    
       isSink = appModule->par("isSink");
    else
	   isSink = false;
     
	// no info on sink at initialization
	mySink.id = -1;
	mySink.x = 0;
	mySink.y = 0;

	declareOutput("GPSR Packets received");
	declareOutput("GPSR Packets sent");
	declareOutput("GPSR Packets forwarded");
    
	// *** 050714
	//path2[16][16] = new list <int>();
	//std::vector<std::list<int[]>> *path ;

	// we will only send HELLO message if the node's coordinates are set
 
	
	//GpsrRoutingControlCommand *cmd = new GpsrRoutingControlCommand("GPSR set node pos", NETWORK_CONTROL_COMMAND);
//cmd->setGpsrRoutingCommandKind(SET_GPSR_NODE_POS);
//cmd->setDouble1(srcX);
//cmd->setDouble2(srcY);
//toNetworkLayer(cmd);
	
		//if (self == 0){
	  //totalSNnodes = getParentModule()->getParentModule()->getParentModule()->par("numNodes");
	  //totalSNnodes = 10;
	  //totalSNnodes = (int)getParentModule()->getParentModule()->getParentModule()->par("numNodes");
	  double distArr[16][16];	
	  //double powrArr[16][16];
	  double enrgArr[16][16];
	  int x1, x2, y1, y2;
	  double distxy;
	  //double distArr[totalSNnodes][totalSNnodes];
	  //double distArr[10][10];    
	  //double PLd; // path loss at distance dist, in dB
	  for (int i = 0; i < 16; i++)
	      for (int j = 0; j < 16; j++) {
		x1 = (int)getParentModule()->getParentModule()->getParentModule()->getSubmodule("node",i)->par("xCoor");
		y1 = (int)getParentModule()->getParentModule()->getParentModule()->getSubmodule("node",i)->par("yCoor");
		x2 = (int)getParentModule()->getParentModule()->getParentModule()->getSubmodule("node",j)->par("xCoor");
		y2 = (int)getParentModule()->getParentModule()->getParentModule()->getSubmodule("node",j)->par("yCoor");
		
		//trace() << "x1=" << x1 << " y1=" << y1 << " x2=" << x2 << " y2=" << y2;
		
		distxy = distance(x1, y1, x2, y2);
		//distxy = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
		
		distArr[i][j] = distxy;
		//enrgArr[i][j] = pow(distxy, 2.0);
		//trace() << "Distance From Node : " << i << " to Node : " << j << " is : " << distxy;
		if (distxy < 8)
		  powrArr[i][j] = 1; // power level = -15dbm
		  
		else if (distxy < 23)
		  powrArr[i][j] = 4; // power level = -7dbm
		  
		else if (distxy < 34) 
		  powrArr[i][j] = 9; // power level = -3dbm
		 
		else
		  powrArr[i][j] = 999; // infinite
		trace() << "Power From Node : " << i << " to Node : " << j << " is : " << powrArr[i][j];
	      }
	    for (int n = 0;n < 16; n++){
	      //int i;
	      //path[n][n] = new list<int>[];
	      // djikstra
	      dijkstra(powrArr, n);
	    }
	    
	/* Go through the list of nodes that were affected
	 *  by this transmission. *it1 holds the node ID
	 */
		list <int>::iterator it1;
		list <int>::iterator it5;
		//list <int>::iterator it1;
		//list <int>::iterator it2;
		/*for (int j = 0; j < 16; j++) 
		  for (int k = 0; k < 16; k++){
		    for (it1 = path[j][k].begin();
			    it1 != path[j][k].end(); it1++) {
			    trace() << "The path from: " << j << " to: " << k << " is: " <<  *it1;
			    //sit2 = pathd[j][k].pop_front();
			    //trace() << "The dist from: " << j << " to: " << k << " is: " <<  *it2;
		    }
		      //trace() << " \n"; //for it1
		    /*for (it2 = pathd[j][k].begin();
			    it2 != pathd[j][k].end(); it2++) {
			    trace() << "The dist from: " << j << " to: " << k << " is: " <<  *it2;
		    }*/
		    //sort(pathd[j][0].begin(), pathd[j][k].end());
		    //}
		    // iterate values from dist list and push them to a sorting list
		    //for (int l = 0; l < 16; l++)
		      for (int k = 0; k <16; k++)
			for (it1 = pathd[0][k].begin();
			  it1 != pathd[0][k].end(); it1++) {
			    //int f = int(*it3);			    
			    paths[k].push_front(int(*it1));
			    paths[k].push_back(k);
			    //trace() << "The dist from: "  << k << paths[k].back() << " to: " << " is: " <<  *it1;
		    }
		    
		    // sort list
		    for(int i=0; i<16; i++)
		    {
		      for(int j=i+1; j<16; j++)
		      {
		      if(paths[i].front() > paths[j].front())
			{
			  int tempf=paths[i].front();
			  int tempb=paths[i].back();
			  paths[i].front()=paths[j].front();
			  paths[i].back()=paths[j].back();
			  paths[j].front()=tempf;
			  paths[j].back()=tempb;
			}
		      }
		    }
		    
		    // iterate values from sorted list
		    for (int j = 0; j < 16; j++){
		    //for (it4 = paths[j].begin(); it4 != paths[j].end(); it4++) {
			    //paths[k] = it4;
			    int val = paths[j].front();
			    int nod = paths[j].back();
			    trace() << "The sort from: "  << j << " to: " << nod << " is: " << val;
		    }
		    
		    /*
		    //for (int j = 0; j < 16; j++){
		      if ((int)powrArr[4][5] == 999){
			int j = 7;
			for (it5 = ++path[4][5].begin();
			      it5 != path[4][5].end(); ++it5){ 
			    trace() << "Next hop via: " << *it5;
			  //for (int i = 1; i < path[paths[j].back()][paths[j+1].back()].size(); i++)	
			  j++;
			  paths[j].push_back(int(*it5));
			  trace() << "paths via: " << *it5;
			    //pathst[j+1].push_back(*it4);
			  }
		      }
		      
		      for (int j = 0; j < 32; j++)
			trace() << "sorted node: " << paths[j].back(); 
			*/
		    //}
	    
	/*
	// iterate through already visited nodes
	for (it2 = ++visit.begin(); it2 != visit.end(); ++it2)
	  *it2;
	
	// insert sortest paths for sorted distances
	for (int j = 0; j < 16; j++){	
		if (paths[j].back() == self)
		  if ((int)powrArr[paths[j].back()][paths[j+1].back()] != 999){
		    //nextHop = paths[j+1].back();
		    trace() << "NextHop is: "  << paths[j+1].back();
		    trace() << (int)powrArr[paths[j].back()][paths[j+1].back()];
		    visit.push_back(j);
		    break;
		  }
		  else if ((int)powrArr[paths[j].back()][paths[j+1].back()] == 999){
		    for (it4 = ++path[paths[j].back()][paths[j+1].back()].begin();
			 it4 != path[paths[j].back()][paths[j+1].back()].end(); ++it4){ 
		      trace() << "Next hop via: " << *it4;
		      paths[j+1].push_back(*it4);
		    }
		    //paths[j].push_back(*it4);
		    advance(it4, 2);
		    if (*it4 == self)
		      advance(it4, 1);
		    //nextHop = *it4;
		    trace() << "NextHop is: "  << *it4;
		    break;
		  }
		  else{
		    trace() << "end of trip!";
		    break;
		  }		 
	}*/		    
	  //}
	
	
}

//================================================================
//    timerFiredCallback
//================================================================
void GpsrRouting::timerFiredCallback(int index){
     
    switch(index){
       
      case GPSR_HELLO_MSG_REFRESH_TIMER :{
		  // *** 260614 no HELLO
		  //sendHelloMessage();
          break;  
      }

      case GPSR_HELLO_MSG_EXPIRE_TIMER :{
		  //sendHelloMessage();
          break;  
      }
      
      default: break;
    }
}

//================================================================
//    processBufferedPacket
//================================================================

void GpsrRouting::processBufferedPacket(){
	while (!TXBuffer.empty()) {
		toMacLayer(TXBuffer.front(), BROADCAST_MAC_ADDRESS);
		TXBuffer.pop();
	}
}

//================================================================
//    fromApplicationLayer
//================================================================
void GpsrRouting::fromApplicationLayer(cPacket * pkt, const char *destination){ 
  
	GpsrPacket *dataPacket = new GpsrPacket("GPSR routing data packet", NETWORK_LAYER_PACKET);
        //srcX = nodeMobilityModule->getLocation().x;
        //srcY = nodeMobilityModule->getLocation().y;
	//trace() << "Source on packet X: " << srcX << " Y: " << srcY;
	
// cut 
	
	encapsulatePacket(dataPacket, pkt);
	dataPacket->setGpsrPacketKind(GPSR_DATA_PACKET);
	dataPacket->setSource(SELF_NETWORK_ADDRESS);
	dataPacket->setDestination(destination);

	if (string(destination).compare(BROADCAST_NETWORK_ADDRESS)==0) {
		// if broadcast, just give it to MAC layer
		trace() << "Received data from application layer, final destination: BROADCAST";
		toMacLayer(dataPacket, BROADCAST_MAC_ADDRESS);
		collectOutput("GPSR Packets received", "DATA from Application (broadcast)");
		collectOutput("GPSR Packets sent", "DATA (broadcast)");
		return;
    }
    
	trace() << "Received data from application layer, final destination: " << string(destination);
    
	if (mySink.id==-1) {
		trace() << "Sink coordinates are not set by application yet for sink node " << string(destination);
		trace() << "Sorry, data will be not sent";
		return;
	}
	else
		trace() << "Sink coordinates are set by application for sink node " << mySink.id;

	// normally, the application layer has to give the coordinates of the destination
	// therefore mySink.id should be equal to atoi(destination)
	// we just need the coordinates stored in MySink.x and mySink.y
    //int nextHop = getNextHopGreedy(mySink.x, mySink.y);
	// *** 150614
	//dataPacket->setX_src(self_xCoo);
	//dataPacket->setY_src(self_yCoo);
	//int nextHop = getNextHopEucl(mySink.x, mySink.y);
	int nextHop = getNextHopGCF(self);	
	// set the coordinate of the sink in the data packet so that intermediate node can get it
	dataPacket->setX_dst(mySink.x);
	dataPacket->setY_dst(mySink.y);
	// *** 190714 addition
	//dataPacket->setDestination(nextHop);
	
   //srcX = getParentModule()->getParentModule()->par("xCoor"); 
    //srcY = getParentModule()->getParentModule()->par("yCoor"); 
    //trace() << "Source on packet from app X: " << srcX << " Y: " << srcY;
	//dataPacket->setX_src(srcX);
	//dataPacket->setY_src(srcY);

	if (nextHop != -1) {
		// It exists a closest neighbor to SINK => Greedy Mode
		trace() << "Send data in greedy mode, next hop node " << nextHop << ", final destination: " << string(destination);
		toMacLayer(dataPacket, nextHop);
		collectOutput("GPSR Packets received", "DATA from Application (unicast,greedy)");
		collectOutput("GPSR Packets sent", "DATA (unicast,greedy)");
		return;
    }
	else
	{
		// It don't exist any closest neighbor to SINK => Perimeter Mode
		//nextHop = getNextHopPerimeter(mySink.x, mySink.y);

		if (nextHop != -1) {
			// the rule of the right hand found. It exists a neighbor
			trace() << "Send data in perimeter mode, next hop node: " << nextHop << ", final destination: " << string(destination);
			toMacLayer(dataPacket, nextHop);
			collectOutput("GPSR Packets received", "DATA from Application (unicast,perimeter)");
			collectOutput("GPSR Packets sent", "DATA (unicast,perimeter)");
			return;
		}
		else {
			trace() << "PRESENCE of hole in node " << self <<
				" when trying to forward data (from application layer) to node " << string(destination);
			delete dataPacket;
		}	
    }	
}

//================================================================
//    fromMacLayer
//================================================================
void GpsrRouting::fromMacLayer(cPacket * pkt, int macAddress, double rssi, double lqi){
	
    GpsrPacket *netPacket = dynamic_cast <GpsrPacket*>(pkt);

	if (!netPacket)
		return;

	switch (netPacket->getGpsrPacketKind()) {
 
        // process hello msg
		case GPSR_HELLO_MSG_PACKET: {

			trace() << "Received HELLO from node " << string(netPacket->getSource()) << "(" <<
					netPacket->getX_src() << "," << netPacket->getY_src()<< ")";
			
			// *** addition 190614
			//srcX = netPacket->getX_src();
			//srcY = netPacket->getY_src();
			//trace() << "Source on packet from mac X: " << srcX << " Y: " << srcY;
			collectOutput("GPSR Packets received", "HELLO");
						
			// *** addition 260614 no HELLO
			updateNeighborTable(atoi(netPacket->getSource()), seqHello, netPacket->getX_src(),netPacket->getY_src());
			break;
		}
        
        // process sink address msg
		case GPSR_SINK_ADDRESS_PACKET:{ 
			//processSinkAddress(netPacket);
			break;
		}
        // process data packet
		case GPSR_DATA_PACKET:{ 
 
			collectOutput("GPSR Packets received", "DATA from MAC");
			
			// *** addition 190614
			//srcX = netPacket->getX_src();
			//srcY = netPacket->getY_src();
			//srcX = getParentModule()->getParentModule()->par("xCoor"); 
			//srcY = getParentModule()->getParentModule()->par("yCoor"); 
			//netPacket->setX_src(srcX);
			//netPacket->setY_src(srcY);
			//trace() << "Source on packet from mac X: " << srcX << " Y: " << srcY;			
			
			string dst(netPacket->getDestination());
			string src(netPacket->getSource());
			
			if ((dst.compare(BROADCAST_NETWORK_ADDRESS) == 0))
				trace() << "Received data from node " << src << " by broadcast";			
			else
				trace() << "Received data from node " << src << ", final destination: " << dst;
			
			//updateNeighborTable(atoi(netPacket->getSource()), seqHello, netPacket->getX_src(),netPacket->getY_src());

			processDataPacketFromMacLayer(netPacket);
			break;
		}

        default: return;
	}
}

//================================================================
//    finishSpecific
//================================================================
void GpsrRouting::finishSpecific() {
    trace() << self << "Finish Specific";
}

//================================================================
//    sendHelloMsg
//================================================================
void GpsrRouting::sendHelloMessage(){

    GpsrPacket *helloMsg = new GpsrPacket("GPSR hello message packet", NETWORK_LAYER_PACKET);
    helloMsg->setGpsrPacketKind(GPSR_HELLO_MSG_PACKET);
    helloMsg->setX_src(self_xCoo);		
    helloMsg->setY_src(self_yCoo);
    helloMsg->setSource(SELF_NETWORK_ADDRESS);
    helloMsg->setDestination(BROADCAST_NETWORK_ADDRESS);
    // *** 260614 no HELLO 
    //toMacLayer(helloMsg, BROADCAST_MAC_ADDRESS);

	trace() << "Broadcast HELLO(" << self_xCoo << "," << self_yCoo << ") seq=" << seqHello;

	collectOutput("GPSR Packets sent", "HELLO");

	seqHello++;
	setTimer(GPSR_HELLO_MSG_REFRESH_TIMER, helloInterval);
}

//================================================================
//    processDataPacket
//================================================================
void GpsrRouting::processDataPacketFromMacLayer(GpsrPacket* pkt){

    string dst(pkt->getDestination());
    string src(pkt->getSource());

	// if the node is the destination
	if ((dst.compare(SELF_NETWORK_ADDRESS) == 0) || (self == mySink.id)) {
		trace() << "Received data for myself (routing unicast) from MAC, send data to application layer. Source node: " << src;
		collectOutput("GPSR Packets received", "final from MAC");
#ifdef DEBUG_OUTPUT_LEVEL2		
		collectOutput("GPSR Packets received", atoi(src.c_str()), "final from MAC");
#endif		
		toApplicationLayer(pkt->decapsulate());
		return;
    } 

	// if the node is the destination by broadcast, we do not forward it
	if ((dst.compare(BROADCAST_NETWORK_ADDRESS) == 0)) {
		trace() << "Received data (routing broadcast) from MAC, send data to application layer. Source node: " << src;
		collectOutput("GPSR Packets received", "broadcast from MAC");
#ifdef DEBUG_OUTPUT_LEVEL2
		collectOutput("GPSR Packets received", atoi(src.c_str()), "broadcast from MAC");
#endif
		toApplicationLayer(pkt->decapsulate());
		return;
	}

	// otherwise, the node has received a message in unicast (through MAC layer) but is not the final destination
	// need to find how to reach the final destination which coordinates' are stored in the incoming packet
	//int nextHop = getNextHopGreedy(pkt->getX_dst(), pkt->getY_dst());
	// *** 150614
	//srcX = pkt->getX_src();
	//srcY = pkt->getY_src();
	
	//trace() << "Source on packet X: " << srcX << " Y: " << srcY;
	//srcX = getParentModule()->getParentModule()->par("xCoor"); 
	//srcY = getParentModule()->getParentModule()->par("yCoor"); 
	//trace() << "Source on packet from packet X: " << srcX << " Y: " << srcY;
	        
	//int nextHop = getNextHopEucl(pkt->getX_dst(), pkt->getY_dst());
	//updateNeighborTable(atoi(pkt->getSource()), seqHello, pkt->getX_src(),pkt->getY_src());
	int nextHop = getNextHopGCF(self);
	trace () <<"NextHop from Mac is:" << nextHop;
	int pos = -1;
	int tblSize = (int)neighborTable.size();
	for (int i = 0; i < tblSize; i++)
		if (neighborTable[i].id == nextHop) {
			pos = i;
			break;
        }
        srcX = neighborTable[pos].x;
	srcY = neighborTable[pos].y; 
	trace() << "Source on packet from packet X: " << srcX << " Y: " << srcY;


	// duplicate the packet because we are going to forward it
	GpsrPacket *netPacket = pkt->dup();
	//netPacket->setX_src(srcX);
	//netPacket->setY_src(srcY);

	if (nextHop != -1) {
		// It exists a closest neighbor to SINK => Greedy Mode
		trace() << "Received data (routing unicast) from MAC, forward data in greedy mode. Source node: " << src
				<< ", next hop node: " << nextHop << ", final destination: " << dst;
        toMacLayer(netPacket, nextHop);
		collectOutput("GPSR Packets forwarded","greedy");
        return;
     }
	 else {
		// It don't exist any closest neighbor to SINK => Perimeter Mode
		//nextHop = getNextHopPerimeter(pkt->getX_dst(), pkt->getY_dst());

		if (nextHop != -1) {
			// the rule of the right hand  found. It exists a neighbor
			trace() << "Received data from MAC, forward data in perimeter mode. Source node: " << src
				<< ", next hop node: " << nextHop << ", final destination: " << dst;
			toMacLayer(netPacket, nextHop);
			collectOutput("GPSR Packets forwarded","perimeter");
			return;
        }
        else
			trace() << "PRESENCE of hole in node " << self <<
				" when trying to forward data (from MAC) from node " << src << " to node " << dst;
     }		          
}

//================================================================
//    updateNeighborTable
//================================================================
void GpsrRouting::updateNeighborTable(int nodeID, int theSN, int x_node, int y_node) {

	int pos = -1;
	int tblSize = (int)neighborTable.size();
	
	for (int i = 0; i < tblSize; i++)
		if (neighborTable[i].id == nodeID) {
			pos = i;
			break;
        }

	// it's a new neighbor
	if (pos == -1) {
		GPSR_neighborRecord newRec;

		newRec.id = nodeID;
		//newRec.id = getParentModule()->getParentModule()->getParentModule()->getSubmodule("node",i);
		newRec.x = x_node;
		newRec.y = y_node;
		// *** Addition 180612
		//newRec.x = (int)getParentModule()->getParentModule()->getSubmodule("node",nodeID)->getSubmodule("Application")->par("xCoordinate").doubleValue() ;
		//newRec.y = (int)getParentModule()->getParentModule()->getSubmodule("node",nodeID)->getSubmodule("Application")->par("yCoordinate") ;
		//newRec.x = (int)(int)node_xy->getLocation().x;
		//newRec.x = (int)nodeModule->getLocation().x;
		//newRec.y = (int)nodeModule->getLocation().y;
		newRec.ts = simTime().dbl();
		newRec.timesRx = 1;

	    neighborTable.push_back(newRec);

		// print the last item
		trace() << "New neighbor for node " << self << " : node "<< neighborTable[(int)neighborTable.size()-1].id;

		/*trace() << "id:" << neighborTable[pos].id << " x:" << neighborTable[pos].x << " y:" << neighborTable[pos].y << " timestamp:" << neighborTable[pos].ts << " times Rx:" << neighborTable[pos].timesRx << " received packets:" << neighborTable[pos].receivedPackets << endl;
                */
	} else {

		//it's an already known neighbor
		neighborTable[pos].x = x_node; // updating of location
		neighborTable[pos].y = y_node;
		neighborTable[pos].ts = simTime().dbl();
		neighborTable[pos].timesRx++;
	}
 
	trace() << "Neighbors list of node " << self << ":";

	tblSize = (int)neighborTable.size();

	for (int j = 0; j < tblSize; j++)
		trace() << "Node " << neighborTable[j].id << "(" << neighborTable[j].x << "," << neighborTable[j].y <<
				"). Received " << neighborTable[j].timesRx << " HELLO from it.";

	trace() << "--------------";
}

bool comp(const int& num1, const int& num2) {
  return num1 > num2;  
}

//================================================================
//    *** 220614 - dijkstra by Stavros Kalapothas
//================================================================
// A utility function to find the vertex with minimum distance value, from
// the set of vertices not yet included in shortest path tree
int  minDistance(int dist[], bool sptSet[])
{
   // Initialize min value
   int min = INT_MAX, min_index;
 
   for (int v = 0; v < V; v++)
     if (sptSet[v] == false && dist[v] <= min)
         min = dist[v], min_index = v;
 
   return min_index;
}
 
// A utility function to print the constructed distance array
void  GpsrRouting::printSolution(int dist[], int parent[], int n)
{

  list <int>::iterator it3;

  ///path.push_back(make_pair(n, i)); 
  trace() << "Vertex   Distance from Source\n"; 
  //pathd[][] = new list<pathd[][]>;
   for (int i = 0; i < V; i++){
     //for (int i = 0; i < sizeof(parent); i++){
    int curr =  parent[i]; 
    trace() << i << " , " << parent[i] << "\t\t " << dist[i];
    pathd[n][i].push_front(dist[i]);
    path[n][i].push_front(i);  
    path[n][i].push_front(parent[i]);
     while (curr != -1){
       
       //int i;
       //int j=parent[i];
       //parent[i]=parent[j];
     
     if (curr == n)
       break;
     curr = parent[curr];
     trace() << " par: " << curr;
     path[n][i].push_front(curr);
      //if(parent[i] != 0) printSolution(dist, parent, n);
      //printPath(parent);
      //enrgArr[i][j] = dist[i];
      //trace() << "Energy all " << enrgArr[i][j];
     
     
     // recursive implementation fails
     //for (int i = 0; i < V; i++){
    //int i;
    //trace() << parent[V];
    //if (parent[V] != -1)
    //printSolution(parent);
    //     trace() << parent << " , ";
     }
   }
   		    // extra path for out of reach 999
		    for(int j=0; j<16; j++)
		      {
		      if ((int)powrArr[n][j] == 999){
			for (it3 = path[n][j].begin();
			      it3 != path[n][j].end(); it3++) 
			    trace() << "Next 999 via: " << *it3; 
			    visit[n][j].push_front(int(*it3));
			//trace() << "wtf3 " << path[n]n][j].back();
			}
			trace() << " \n";
		      }
}

/*
void  GpsrRouting::printPath(int dest) {
	if (parent[dest] != -1)
		printPath(parent[dest]);
	printf("%d ", dest);
}
*/

// Funtion that implements Dijkstra's single source shortest path algorithm
// for a graph represented using adjacency matrix representation
void  GpsrRouting::dijkstra(double distArr[V][V], int src)
{
     int parent[V]; // Array to store constructed MST
     int dist[V];     // The output array.  dist[i] will hold the shortest
                      // distance from src to i
 
     bool sptSet[V]; // sptSet[i] will true if vertex i is included in shortest
                     // path tree or shortest distance from src to i is finalized
           
     parent[0] = -1; // First node is always root of MST
 
     // Initialize all distances as INFINITE and stpSet[] as false
     for (int i = 0; i < V; i++)
        dist[i] = INT_MAX, sptSet[i] = false;
 
     // Distance of source vertex from itself is always 0
     dist[src] = 0;
 
     // Find shortest path for all vertices
     for (int count = 0; count < V-1; count++)
     {
       // Pick the minimum distance vertex from the set of vertices not
       // yet processed. u is always equal to src in first iteration.
       int u = minDistance(dist, sptSet);
 
       // Mark the picked vertex as processed
       sptSet[u] = true;
 
       // Update dist value of the adjacent vertices of the picked vertex.
       for (int v = 0; v < V; v++)
 
         // Update dist[v] only if is not in sptSet, there is an edge from
         // u to v, and total weight of path from src to  v through u is
         // smaller than current value of dist[v]
         if (!sptSet[v] && distArr[u][v] && dist[u] != INT_MAX
                                       && dist[u]+distArr[u][v] < dist[v]){
             parent[v]  = u, dist[v] = dist[u] + distArr[u][v]; 
	  trace() << "count: " << count << " parent: " << u << " dist: " << dist[v];
	 
	 }
	
     }
 
     // fill in and print the constructed distance array
     //printSolution(dist, parent, V);
     //for (int i = 0; i < V; i++)
     printSolution(dist, parent, src);
}

//================================================================
//   getNextHopGCF  *** Addition 120714 by Stavros Kalapothas 
//================================================================
int GpsrRouting::getNextHopGCF(int nodeID){
  
   int nextHop = -1; double dist = 0;
   int tblSize = (int)neighborTable.size();
   list <int>::iterator it4;
   list <int>::iterator it2;
   list <int>::iterator it3;

   //initializing the minimal distance as my distance to sink
   trace() << "Node "<< self << "(" << self_xCoo << "," << self_yCoo << ")"; 
   trace() << "flag: " << flag[0];

    // iterate through already visited nodes
    //for (it2 = visit.begin(); it2 != visit.end(); ++it2){
      //trace() << "Visited nodes: " << *it2;  
   // && nodeID == *it2
   /*
   for (int j = 0; j < 16; j++){
     if ((int)powrArr[paths[j].back()][paths[j+1].back()] == 999)
	for (it3 = ++path[paths[j].back()][paths[j+1].back()].begin();
			      it3 != path[paths[j].back()][paths[j+1].back()].end(); ++it3){		    
		      paths[j].push_back(int(*it3));
		      //paths[j].insert(path
		    //trace() << "wtf3 " << *it3;
		}
   }*/		
		
	// iterate values from sorted list
	for (int j = 0; j < 16; j++){
	  //for (it4 = ++path[paths[j].back()][paths[j+1].back()].begin();
			//it4 != path[paths[j].back()][paths[j+1].back()].end(); ++it4)
	       //int x = *it4;
		if (paths[j].back() == nodeID && (int)powrArr[paths[j].back()][paths[j+1].back()] != 999){
		  //trace() << "wtf2 " << *it4;
		  //if ((int)powrArr[paths[j].back()][paths[j+1].back()] != 999){
		    //advance(it4, flag[0]);
		    nextHop = paths[j+1].back();
		    //nextHop = path[paths[j].back()][paths[j+1].back()].back();
		    trace() << "NextHop is: "  << nextHop;
		    //trace() << "Visit is: " << visit[1].back();
		    trace() << (int)powrArr[paths[j].back()][paths[j+1].back()];
		    trace() << " wtf " << path[paths[j].back()][paths[j+1].back()].front();
		    //visit[1].clear();
		    //visit.push_back(1);
		    //pathf = 1;
		    flag[0]=j;
		    break;
		  }
		  else{
		      for (it2 = ++path[paths[j].back()][paths[j+1].back()].begin();
			  it2 != path[paths[j].back()][paths[j+1].back()].end(); ++it2) {
			trace() << "Next hop2 via: " << *it2;
		      for (int i = 1; i < path[paths[j].back()][paths[j+1].back()].size(); i++)
		      paths[j+i].push_back(int(*it2));
		      }
		      advance(it2, 2);
		      //if (*it2 == self)
			//advance(it2, 1);
		      nextHop = paths[j+1].back();
		      //visit[paths[j].back()][paths[j+1].clear();
		      //visit[paths[j].back()][paths[j+1]push_back(int(*it4));		      
		      //flag[0]++;
		      j++;
		      flags = 1;
		      trace() << "end of trip!";
		      break;
		    }
		//}
	}
	//}
    
   return nextHop;
   
}

//================================================================
//   getNextHopGreedy
//================================================================
int GpsrRouting::getNextHopGreedy(int x_Sink, int y_Sink){
  
   int nextHop = -1; double dist = 0;
   int tblSize = (int)neighborTable.size();

   //initializing the minimal distance as my distance to sink
   trace() << "Node "<< self << "(" << self_xCoo << "," << self_yCoo << ")";

   double minDist = distance(self_xCoo, self_yCoo, x_Sink, y_Sink);
   //trace()<< "Distance ("<< self <<", Sink)" << " = " << minDist << endl; 

   for (int i = 0; i < tblSize; i++) {
		dist = distance(neighborTable[i].x, neighborTable[i].y, x_Sink, y_Sink);
		trace() << "Distance ("<< neighborTable[i].id <<", Sink)" << " = " << dist;

		if (dist < minDist) {
			minDist = dist;
			nextHop = neighborTable[i].id;
		}
   }

   return nextHop;
}

//================================================================
//   getNextHopEucl  *** Addition 150614 by Stavros Kalapothas
//================================================================
int GpsrRouting::getNextHopEucl(int x_Sink, int y_Sink){
  
   int nextHop = -1; double dist = 0;
   int tblSize = (int)neighborTable.size();

   //initializing the minimal distance as my distance to sink
   trace() << "Node "<< self << "(" << self_xCoo << "," << self_yCoo << ")";

   double minDist = distance(self_xCoo, self_yCoo, x_Sink, y_Sink);
   //trace()<< "Distance ("<< self <<", Sink)" << " = " << minDist << endl; 
   trace() << "Source on routing X: " << srcX << " Y: " << srcY;

   for (int i = 0; i < tblSize; i++) {
		if ((neighborTable[i].x != srcX) && (neighborTable[i].y != srcY)) {
		dist = distance(neighborTable[i].x, neighborTable[i].y, self_xCoo, self_yCoo);
		trace() << "Distance ("<< neighborTable[i].id <<", Self)" << " = " << dist;
		}
		
		if (dist < minDist) {
			minDist = dist;
			nextHop = neighborTable[i].id;
			//neighborTable.erase(neighborTable.begin()+i);
		}
   }

   return nextHop;
}
//================================================================
//    getNextHopPerimeter
//================================================================
int GpsrRouting::getNextHopPerimeter(int x_Sink, int y_Sink) {
    // NOT IMPLEMENTED YET
    return -1;
}

//================================================================
//    distance
//================================================================
double GpsrRouting::distance(int x1, int y1, int x2, int y2) {

    return sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));	
}

// will handle interaction between the application layer and the GPRS module in order to pass parameters such as
// the node's position
void GpsrRouting::handleNetworkControlCommand(cMessage *msg) {

	GpsrRoutingControlCommand *cmd = check_and_cast <GpsrRoutingControlCommand*>(msg);

	switch (cmd->getGpsrRoutingCommandKind()) {

		case SET_GPSR_NODE_POS: {

			self_xCoo = cmd->getDouble1();
			self_yCoo = cmd->getDouble2();
			isCoordinateSet = true;

			trace() << "Application layer has set node's position for to (" << self_xCoo << "," << self_yCoo << ")";

			// normally, this is the first HELLO message
			if (isCoordinateSet) {
				  // *** 260614 no HELLO
				  sendHelloMessage();
			 }

			break;
		}

		case SET_GPSR_SINK_POS: {

			mySink.x = (int)cmd->getDouble1();
			mySink.y = (int)cmd->getDouble2();
			mySink.id = cmd->getInt1();

			trace() << "Application layer has set sink's position for next transfers SINK_" << mySink.id << "(" << mySink.x << ","
					<< mySink.y << ")";

			break;
		}
	}
	// don't delete the message since it will get deleted by the VirtualRouting class
}
