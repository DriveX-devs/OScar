#include <iostream>
#include <string>
// TCLAP headers
#include "tclap/CmdLine.h"

int main (int argc, char *argv[]) {
	std::string dissem_vif = "wlan0";

	// Parse the command line options with the TCLAP library
	try {
		TCLAP::CmdLine cmd("The Open CA Basic Service implementatiuon", ' ', "0.1");

		// Arguments: short option, long option, description, is it mandatory?, default value, type indication (just a string to help the user)
		TCLAP::ValueArg<std::string> vifName("I","interface","Broadcast dissemination interface",false,"wlan0","string");
		cmd.add(vifName);

		// TCLAP::ValueArg<std::string> queueArg("Q","queue","Broker queue or topic",false,"topic://5gcarmen.examples","string");
		// cmd.add(queueArg);

		// TCLAP::ValueArg<std::string> gntstpropArg("T","gn-tst-prop","Name of the amqp gn-timestamp property",false,"gn_ts","string");
		// cmd.add(gntstpropArg);

		// TCLAP::ValueArg<int> portArg("P","comm-port","Port for the UDP communication with ms-van3t",false,20000,"int");
		// cmd.add(portArg);

		// TCLAP::SwitchArg skipGNArg("S","skip-gn","Specify this option to send only Facilities Layer messages, instead of full ITS messages (Facilities layer + GeoNetworking + BTP). Warning! Experimental feature (it will work when only CAMs are sent)!");
		// cmd.add(skipGNArg);

		cmd.parse(argc,argv);

		dissem_vif=vifName.getValue();

		std::cout << "[INFO] CAM dissemination interface: " << dissem_vif << std::endl;
	} catch (TCLAP::ArgException &tclape) { 
		std::cerr << "TCLAP error: " << tclape.error() << " for argument " << tclape.argId() << std::endl;
	}

	return 0;
}