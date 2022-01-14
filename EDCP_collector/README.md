# Sample EDCP Collector

This directory contains the source code of a sample C++ EDCP (Extra Device Communication Protocol) Collector, i.e., a program than can run as a service on an extra computation device, connected to the device running OCABS through a network and providing additional information about its current computational load.
This sample EDCP Collector is thought to run on Nvidia Jetson boards and can gather CPU and GPU load metrics, together with the amount of free RAM in MB.
It can then report this data to any client sending a proper EDCP REQUEST (i.e., either OCABS, when sending Enhanced CAMs, or any custom client, like the included sample Python 3 client).
 
You can compile the EDCA Collector on the target extra computation device with:
`g++ -o edcp_collector edcp_collector.cpp`
