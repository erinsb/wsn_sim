# wsn_sim
Discrete simulator for Wireless Sensor Network applications.

Based on the Nordic Semiconductor nRF51 radio, this framework simulates real conditions in wireless sensor networks, and features packet collisions, accurate transmit times, random packet drops, realistic power consumption models and graphical representation of power consumption in devices, clock drift, real BLE packets.

The simulator displays power consumption graphs in OpenGL by the use of [Matplot++](https://code.google.com/p/matplotpp/), an open source C++ graphing library based on the Matlab plotting functionality. The Matplot++ framework is included in the repo, with GLUT headers and libraries for Windows. 

The framework can also deploy [GraphViz](www.graphviz.org) files for graphical topology representation, but GraphViz itself must be downloaded from their website (it's free).

## Why was this made?
The project is part of Trond Einar Snekvik's Master Thesis at NTNU. All existing simulator solutions I could find were either aimed at less power constrained systems, had too much fluff and bloat or didn't include proper power analyzation, so I made my own.

## What has been done?
The framework has additional Bluetooth Low Energy packets and a BLE advertiser/scanner example included. In addition, a novel Meshing solution is partly implemented.
