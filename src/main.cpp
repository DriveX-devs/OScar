#include <iostream>
#include <string>
#include <climits>
#include <cfloat>
#include <unistd.h>
// TCLAP headers
#include "tclap/CmdLine.h"
// VDP GPS Client
#include "gpsc.h"

#define INVALID_LONLAT -DBL_MAX
#define GNSS_DEFAULT_PORT 3000

int main (int argc, char *argv[]) {
	std::string dissem_vif = "wlan0";
	std::string gnss_device = "localhost";
	long gnss_port = 3000; // Using 3000 as default port, in our case
	double fixed_lat = -DBL_MAX; // [deg]
	double fixed_lon = -DBL_MAX; // [deg]
	double fixed_speed_ms = -DBL_MAX; // [m/s]
	double fixed_heading_deg = -DBL_MAX; // [deg]

	// Parse the command line options with the TCLAP library
	try {
		TCLAP::CmdLine cmd("The Open CA Basic Service implementatiuon", ' ', "0.1");

		// Arguments: short option, long option, description, is it mandatory?, default value, type indication (just a string to help the user)
		TCLAP::ValueArg<std::string> vifName("I","interface","Broadcast dissemination interface",false,"wlan0","string");
		cmd.add(vifName);

		TCLAP::ValueArg<std::string> GNSSDevArg("D","gnss-device","GNSS device to be used",false,"localhost","string");
		cmd.add(GNSSDevArg);

		TCLAP::ValueArg<long> GNSSPortArg("P","gnss-port","Port to be used to connect to the GNSS device",false,GNSS_DEFAULT_PORT,"integer");
		cmd.add(GNSSPortArg);

		TCLAP::ValueArg<double> fixedLat("L","fixed-lat","Force a fixed latitude value when testing without a GNSS device",false,-DBL_MAX,"float (deg)");
		cmd.add(fixedLat);

		TCLAP::ValueArg<double> fixedLon("l","fixed-lon","Force a fixed longitude value when testing without a GNSS device",false,-DBL_MAX,"float (deg)");
		cmd.add(fixedLon);

		TCLAP::ValueArg<double> fixedSpeed("s","fixed-speed","Force a fixed speed value when testing without a GNSS device",false,-DBL_MAX,"float (m/s)");
		cmd.add(fixedSpeed);

		//TCLAP::ValueArg<double> fixedHeading("h","fixed-heading","Force a fixed heading value when testing without a GNSS device",false,-DBL_MAX,"float (deg)");
		//cmd.add(fixedHeading);

		// TCLAP::SwitchArg skipGNArg("S","skip-gn","Specify this option to send only Facilities Layer messages, instead of full ITS messages (Facilities layer + GeoNetworking + BTP). Warning! Experimental feature (it will work when only CAMs are sent)!");
		// cmd.add(skipGNArg);

		cmd.parse(argc,argv);

		dissem_vif=vifName.getValue();

		gnss_device=GNSSDevArg.getValue();
		gnss_port=GNSSPortArg.getValue();

		fixed_lat=fixedLat.getValue();
		fixed_lon=fixedLon.getValue();
		fixed_speed_ms=fixedSpeed.getValue();
		//fixed_heading_deg=fixedHeading.getValue();

		std::cout << "[INFO] CAM dissemination interface: " << dissem_vif << std::endl;
	} catch (TCLAP::ArgException &tclape) { 
		std::cerr << "TCLAP error: " << tclape.error() << " for argument " << tclape.argId() << std::endl;

		return 1;
	}

	// VDP (Vehicle Data Provider) GPS Client object test
	try {
		VDPGPSClient vdpgpsc(gnss_device,gnss_port);

		while (1) {
			VDPGPSClient::CAM_mandatory_data_t CAMdata;

			std::cout << "[INFO] VDP GPS Client test: getting GNSS data..." << std::endl;

			CAMdata = vdpgpsc.getCAMMandatoryData();

			std::cout << "[INFO] VDP GPS Client test result: Lat: " << CAMdata.latitude << " deg - Lon: " << CAMdata.longitude << " deg." << std::endl;

			sleep(1);
		}
	} catch(const std::exception& e) {
		std::cerr << "Error in creating a new VDP GPS Client object: " << e.what() << std::endl;
	}

	return 0;
}