# CastaliaGCF

Routing Algorithm for Mobile Agent (MA) based Wireless Sensor Network (WSN) for Castalia Simulator

## Information

A research work for WSN based on Mobile Agents. My main task is to evaluate the known routing protocols like GCF, LCF and others, using the Castalia WSN Simulator.

Based on the implementation of GPSR which really helped me to understand more deeply the Castalia's* architecture.

The algorithm is pre-calculating the Shortest Path in a sink node using Dijkstra that I 've implemented in Castalia. I 've created the getNextHopGCF() function as a basis to compute the shortest path while I also take into account all nodes' power levels apart from geo.x.y distances as it is considered a critical factor of performance in designing routing algorithms on WSN.

*Castalia is a Wireless Sensor Network Simulator <https://castalia.forge.nicta.com.au/index.php/en/>

## Licence

Copyright (c) 2012 Stavros Kalapothas (aka Stevaras) <stavros@ubinet.gr>.
It is free software, and may be redistributed under the terms of the GNU Licence.
